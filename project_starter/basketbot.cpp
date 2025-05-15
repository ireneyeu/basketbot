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


bool simulation = true;



// States 
enum State {
	POSTURE = 0,
	WAITING,
	MOTION_UP,
	MOTION_DOWN,
	TEST1,
	TEST2,
	TEST3
};

int main() {

	if (simulation){
		cout << "SIMULATION TRUE" << endl;
		JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
	} else{
		cout << "SIMULATION FALSE" << endl;
		JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";
		JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
		JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
	}

	// Location of URDF files specifying world and robot information
	static const string robot_file = string(BASKETBOT_URDF_FOLDER) + "/panda/panda_arm_box.urdf";

	// initial state 
	int state = POSTURE;
	string controller_status = "2"; // "1" = test up down, "2" = test orientation speed, "3" = test up down with orientation, "4" = waiting
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// // Define Information
	// Ball information
	Vector3d ball_position;
	Vector3d ball_velocity;
	Vector3d ball_vel_des;

	// Robot states
	Vector3d ee_pos;
	Vector3d ee_vel;
	Matrix3d ee_ori;
	VectorXd robot_q(7);
	VectorXd robot_dq(7);
	Vector3d EE_FORCES;
	Vector3d EE_MOMENTS;
	Vector3d ee_forces;
	Vector3d ee_moments;

	// Robot initial states
	Vector3d ee_pos_init;
	Vector3d ee_vel_init;
	Matrix3d ee_ori_init;
	VectorXd robot_q_init(7);
	VectorXd robot_dq_init(7);

	// Robot desired states
	Vector3d ee_pos_desired;
	Vector3d ee_vel_desired;
	Matrix3d ee_ori_desired;
	VectorXd q_desired(7);

	// Robot Gains
	VectorXd kp_xyz;
	VectorXd kv_xyz;
	VectorXd kp_vel_xyz;
	VectorXd kv_vel_xyz;
	VectorXd kp_ori_xyz;
	VectorXd kv_ori_xyz;

	float dribble_count = 0.0;
	bool ball_valid;

	// // Assign initial values
	// Initial robot state
	//ee_pos_init << 0.0, 0.575, 0.328;

	ee_pos_init << 0.15, 0.6, 0.4;
	ee_vel_init = Vector3d::Zero();
	ee_ori_init << 1, 0, 0,
					0, -1, 0,
					0, 0, -1;
	robot_q_init << 0.0, -25.0, 0.0, -135.0, 0.0, 105.0, 0.0;
	robot_q_init *= M_PI/180.0;
	robot_dq_init = VectorXd::Zero(7);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);

	// Try reading Redis keys; fallback to defaults if not present (in order to launch controller first)
	try {
		VectorXd q = redis_client.getEigen(JOINT_ANGLES_KEY);
		VectorXd dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
		ball_position = redis_client.getEigen(BALL_POSITION_KEY);
		ball_velocity = redis_client.getEigen(BALL_VELOCITY_KEY);
		EE_FORCES = redis_client.getEigen(EE_FORCES_KEY);
		EE_MOMENTS = redis_client.getEigen(EE_MOMENTS_KEY);
		robot->setQ(q);
		robot->setDq(dq);
	} catch (const std::exception& e) {
		std::cerr << "Warning: redis values empty." << "\nSetting default joint values in Redis." << std::endl;
		// Set default place-holding values
		robot->setQ(robot_q_init);
		robot->setDq(robot_dq_init);
		ball_position << 0.575, 0.0, 0.0;
		ball_velocity << 0.0, 0.0, 3.0;
		EE_FORCES = Vector3d::Zero();
		EE_MOMENTS = Vector3d::Zero();
		//redis_client.setEigen(JOINT_ANGLES_KEY, robot_q_init);
		//redis_client.setEigen(JOINT_VELOCITIES_KEY, robot_dq_init);
		redis_client.setEigen(BALL_POSITION_KEY, ball_position);
		redis_client.setEigen(BALL_VELOCITY_KEY, ball_velocity);
		redis_client.setEigen(EE_FORCES_KEY, EE_FORCES);
		redis_client.setEigen(EE_MOMENTS_KEY, EE_MOMENTS);
	}

	robot->updateModel();

	// prepare controllers
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	//const Vector3d control_point = Vector3d(0.05, 0, 0.13); // NEED TO CHECK CONTROL POINT
	const Vector3d control_point = Vector3d(0.0, 0, 0.0);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	
	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);

	// Desired robot states and gains
	// Set Gains
	kp_xyz = Vector3d(100.0, 100.0, 100.0);
	kv_xyz = Vector3d(20.0, 20.0, 20.0);
	kp_ori_xyz = Vector3d(100.0, 100.0, 100.0);
	kv_ori_xyz = Vector3d(20.0, 20.0, 20.0);

	pose_task->setPosControlGains(kp_xyz, kv_xyz);
	pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);
	joint_task->setGains(50, 14, 0);

	// Set Goals
	ee_pos_desired = ee_pos_init;
	ee_vel_desired = ee_vel_init;
	ee_ori_desired = ee_ori_init;
	q_desired = robot_q_init;

	q_desired << 1.46298,-0.246285,0.030638,-2.07553,0.0518735,1.84324,0.679142;

	// Set tasks
	pose_task->setGoalPosition(ee_pos_desired);
	joint_task->setGoalPosition(q_desired);	

	cout << "Entering controller loop" << endl;
	cout << "["<< state << "]" << endl;


	// create a loop timer
	runloop = true;
	double control_freq = 1000; // should be 1000
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	//pose_task->enableInternalOtgAccelerationLimited(4.0, 4.0, M_PI/3, M_PI); // OTG LIMITS STUFF
	pose_task->disableInternalOtg();

	double time_start;


	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update ball
		ball_position = redis_client.getEigen(BALL_POSITION_KEY);
		ball_velocity = redis_client.getEigen(BALL_VELOCITY_KEY);

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// robot states
		ee_pos = robot->position(control_link, control_point);

		//cout << ee_pos << endl << endl;
		ee_vel = robot->linearVelocity(control_link, control_point);
		ee_ori = robot->rotation(control_link);
		robot_q = robot->q();
		robot_dq = robot->dq();

		// forces and moments
		EE_FORCES = redis_client.getEigen(EE_FORCES_KEY);
		EE_MOMENTS = redis_client.getEigen(EE_MOMENTS_KEY);
		pose_task->updateSensedForceAndMoment(EE_FORCES, EE_MOMENTS);
		ee_forces = pose_task->getSensedForceControlWorldFrame();
		ee_moments = pose_task->getSensedMomentControlWorldFrame();

		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques();

			// cout << (robot->q() - q_desired).norm() << endl;

			if ((robot->q() - q_desired).norm() < 8e-2) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos_init = ee_pos;
				ee_ori_init = ee_ori;

				pose_task->setGoalPosition(ee_pos);
				time_start = time;

				if (controller_status == "1") {
					state = TEST1;
				} else if (controller_status == "2") {
					state = TEST2;
					cout << AngleAxisd(0, Vector3d::UnitX()).toRotationMatrix() << ee_ori << endl;
				} else if (controller_status == "3") {
					state = TEST3;
				} else if (controller_status == "4") {
					state = WAITING;
				}
			}
		}

		if (state == WAITING) {
			// update task model 

			ee_pos_desired(0) = ball_position(0) ;
			ee_pos_desired(1) = ball_position(1) ;

			ee_ori_desired = ee_ori_init;
			


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
				kp_xyz(2) = 20.0;
				kv_xyz(2) = 20.0;
				kp_ori_xyz(1) = 20.0;
				pose_task->setPosControlGains(kp_xyz, kv_xyz);
				pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);

				ball_vel_des = ball_velocity;

				state = MOTION_UP;
				cout << "["<< state << "]" << endl;
			}
		} else if (state == MOTION_UP) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if (ball_velocity(2) < .1 || abs(ee_pos(2) - ball_position(2)) > 0.17) {
				cout << "MOTION UP TO MOTION DOWN" << endl;

				// clear values for next motion
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				// set up new gains
				kp_xyz(2) = 400.0;
				kv_xyz(2) = 100.0;
				kp_ori_xyz(1) = 200.0;
				pose_task->setPosControlGains(kp_xyz, kv_xyz);
				pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);

				ee_pos_desired = ee_pos_init;
				ee_pos_desired(2) -= 0.1;
				ee_vel_desired << 0, 0, -ball_vel_des(2);
				ee_ori_desired = ee_ori_init;
				q_desired = robot_q_init;

				// pose_task->disableInternalOtg();

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalLinearVelocity(ee_vel_desired);
				pose_task->setGoalOrientation(ee_ori_desired);
				joint_task->setGoalPosition(q_desired);

				state = MOTION_DOWN;
				cout << "["<< state << "]" << endl;
			}	
				
		} else if (state == MOTION_DOWN) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if (abs(ee_pos(2) - ee_pos_desired(2)) < 0.05) {
				cout << "MOTION DOWN TO WAITING" << endl;
				dribble_count += 1.0;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos_desired = ee_pos_init;
				ee_vel_desired = ee_vel_init;
				ee_ori_desired = ee_ori_init;
				q_desired = robot_q_init;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalOrientation(ee_ori_desired);
				joint_task->setGoalPosition(q_desired);

				state = WAITING;
				cout << "["<< state << "]" << endl;
			}
		} else if (state == TEST1) {

			double freq = 4.0;

            ee_pos_desired(2) = ee_pos_init(2) + 0.1 * sin(freq*(time - time_start));

			ee_vel_desired(0) = 0;
			ee_vel_desired(1) = 0;
			ee_vel_desired(2) = 0.1*freq*cos(freq*(time - time_start));

            pose_task->setGoalPosition(ee_pos_desired);
			pose_task->setGoalLinearVelocity(ee_vel_desired);
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
 
            command_torques = pose_task->computeTorques() + joint_task->computeTorques();
        } else if (state == TEST2) {

			double freq = 2.5;
			float theta = -13*M_PI/180.0 + 26.0*M_PI/180.0 * sin(freq*(time - time_start));
			ee_pos_desired = ee_pos_init;
			pose_task->setGoalPosition(ee_pos_desired);
			pose_task->setGoalOrientation(AngleAxisd(theta, ee_ori_init.col(1)).toRotationMatrix() * ee_ori_init);

            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
 
            command_torques = pose_task->computeTorques() + joint_task->computeTorques();
        }

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigen(EE_POSITION_KEY, ee_pos);
		redis_client.setEigen(EE_VELOCITY_KEY, ee_vel);
	}

	timer.stop();
	cout << "Total robot dribble count: " << dribble_count << endl;
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
