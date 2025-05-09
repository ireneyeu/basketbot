/**
 * @file basketbot.cpp
 * @brief Controller file
 * 
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
	WAITING = 0,
	MOTION_UP,
	MOTION_DOWN,
};

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(BASKETBOT_URDF_FOLDER) + "/panda/panda_arm_box.urdf";

	// initial state 
	int state = WAITING;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	// const string control_link = "link7";
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0.20, 0, 0.02); // NEED TO CHECK CONTROL POINT
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	VectorXd kp_xyz = Vector3d(400.0, 400.0, 400.0);
	VectorXd kv_xyz = Vector3d(100.0, 100.0, 100.0);
	pose_task->setPosControlGains(kp_xyz, kv_xyz);
	VectorXd kp_ori_xyz = Vector3d(200.0, 200.0, 200.0);
	VectorXd kv_ori_xyz = Vector3d(30.0, 30.0, 30.0);
	pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);

	// Robot states
	Vector3d ee_pos;
	Vector3d ee_vel;
	Matrix3d ee_ori;
	VectorXd robot_q(dof);
	VectorXd robot_dq(dof);
	Vector3d ee_forces;
	Vector3d ee_moments;

	// Ball information
	Vector3d ball_position;
	Vector3d ball_velocity;
	Vector3d ball_vel_des;


	// Desired states
	Vector3d ee_pos_desired;
	Vector3d ee_vel_desired;
	Matrix3d ee_ori_desired;
	VectorXd q_desired(dof);


	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(50, 14, 0);
	
	// Initial robot state
	Vector3d initial_ee_pos = robot->position(control_link, control_point);
	Matrix3d ee_init_ori;
	ee_init_ori << 1, 0, 0,
					0, -1, 0,
					0, 0, -1;
	VectorXd initial_joint_angles = robot->q();


	cout << "Initial end-effector position: " << initial_ee_pos.transpose() << endl;

	// Desired robot states
	ee_pos_desired = Vector3d(0.7, 0.0, 0.427);
	pose_task->setGoalPosition(ee_pos_desired);
	ee_vel_desired = Vector3d(0.0, 0.0, 0.0);

	q_desired = initial_joint_angles;
	joint_task->setGoalPosition(q_desired);	

	cout << "Entering controller loop" << endl;
	cout << "[WAITING] ball_z - ee_z < 0.15"<< endl;


	// create a loop timer
	runloop = true;
	double control_freq = 1000; // should be 1000
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	pose_task->enableInternalOtgAccelerationLimited(4.0, 4.0, M_PI/3, M_PI);


	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// robot states
		ee_pos = robot->position(control_link, control_point);
		ee_vel = robot->linearVelocity(control_link, control_point);
		ee_ori = robot->rotation(control_link);
		robot_q = robot->q();
		robot_dq = robot->dq();

		// forces and moments
		pose_task->updateSensedForceAndMoment(redis_client.getEigen(EE_FORCES_KEY), redis_client.getEigen(EE_MOMENTS_KEY));
		ee_forces = pose_task->getSensedForceControlWorldFrame();
		ee_moments = pose_task->getSensedMomentControlWorldFrame();

		// ball position and velocity
		ball_position = redis_client.getEigen(BALL_POSITION_KEY);
		ball_velocity = redis_client.getEigen(BALL_VELOCITY_KEY);

		// if (abs(ee_forces(2)) > 0.0000001) {
		// 	cout << ee_forces(2) << endl;
		// }
		// cout << (ee_pos.transpose()-ee_pos_desired.transpose()).norm() << endl;
		// cout << "ball" << ball_position.transpose() << " " << ball_velocity.transpose() << endl;
		
		if (state == WAITING) {
			// update task model 

			ee_pos_desired(0) = ball_position(0) ;
			ee_pos_desired(1) = ball_position(1) ;
			pose_task->setGoalPosition(ee_pos_desired);

			ee_vel_desired(0) = ball_velocity(0);
			ee_vel_desired(1) = ball_velocity(1);
			pose_task->setGoalLinearVelocity(ee_vel_desired);

			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if (abs(ball_position(2) - ee_pos(2)) < 0.15) {
				cout << "Ball Detected" << endl;
				cout << "WAITING TO MOVING UP" << endl;

				// compliant in Z direction
				kp_xyz(2) = 10.0;
				kv_xyz(2) = 10.0;
				// kp_ori_xyz(2) = 0.0;
				pose_task->setPosControlGains(kp_xyz, kv_xyz);
				// pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);

				ball_vel_des = ball_velocity;

				state = MOTION_UP;
			}
		} else if (state == MOTION_UP) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if (ball_velocity(2) < .1) {
				cout << "Motion Up to Motion Down" << endl;
				cout << "[DOWN] ee_z- ee_des_z < 0.1" << abs(ee_pos(2) - ee_pos_desired(2)) <<endl;

				// clear values for next motion
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				// set up new gains
				kp_xyz(2) = 400.0;
				kv_xyz(2) = 100.0;
				kp_ori_xyz(2) = 200.0;
				pose_task->setPosControlGains(kp_xyz, kv_xyz);
				pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);

				ee_vel_desired << 0, 0, -ball_vel_des(2);

				// pose_task->disableInternalOtg();
				cout << "ee vel: " << ee_vel_desired.transpose() << endl;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalLinearVelocity(ee_vel_desired);
				pose_task->setGoalOrientation(ee_init_ori);
				joint_task->setGoalPosition(q_desired);

				state = MOTION_DOWN;
			}	
				
		} else if (state == MOTION_DOWN) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();
			// cout << abs(ee_pos(2) - ee_pos_desired(2)) << endl;

			if (abs(ee_pos(2) - ee_pos_desired(2)) < 0.05) {
				cout << "Motion Down to Waiting" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
				cout << "EE pos des: " << ee_pos_desired.transpose() << endl;

				q_desired = robot_q;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalOrientation(ee_init_ori);
				joint_task->setGoalPosition(q_desired);

				state = WAITING;
			}
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
