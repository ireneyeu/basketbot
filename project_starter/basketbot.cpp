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
	POSTURE = 0, 
	MOTION_UP,
	MOTION_DOWN,
};

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(BASKETBOT_URDF_FOLDER) + "/panda/panda_arm_box.urdf";

	// initial state 
	int state = POSTURE;
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
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	// const string control_link = "link7";
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0.20, 0, 0.02); // NEED TO CHECK CONTROL POINT
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(100, 20, 0);
	pose_task->setOriControlGains(200, 30, 0);

	// Robot states
	Vector3d ee_pos;
	Vector3d ee_vel;
	Matrix3d ee_ori;
	VectorXd robot_q(dof);
	VectorXd robot_dq(dof);
	Vector3d ee_forces;
	Vector3d ee_moments;

	// Desired states
	Vector3d ee_pos_desired;
	Vector3d ee_vel_desired;
	Matrix3d ee_ori_desired;
	VectorXd q_desired(dof);


	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(50, 14, 0);

	// VectorXd q_desired(dof);
	// q_desired.head(7) << 0.0, 30.0, 0.0, -60.0, 0.0, 90.0, 45.0;
	// q_desired.head(7) *= M_PI / 180.0;
	// // q_desired.tail(2) << 0.04, -0.04;
	// joint_task->setGoalPosition(q_desired);

	
	// Initial robot state
	Vector3d initial_ee_pos = robot->position(control_link, control_point);
	cout << "Initial end-effector position: " << initial_ee_pos.transpose() << endl;
	ee_pos_desired = initial_ee_pos;
	pose_task->setGoalPosition(ee_pos_desired);

	VectorXd initial_joint_angles = robot->q();
	q_desired = initial_joint_angles;
	joint_task->setGoalPosition(q_desired);	

	cout << "Establishing posture" << endl;

	// create a loop timer
	runloop = true;
	double control_freq = 1000; // should be 1000
	SaiCommon::LoopTimer timer(control_freq, 1e6);

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

		pose_task->updateSensedForceAndMoment(redis_client.getEigen(EE_FORCES_KEY), redis_client.getEigen(EE_MOMENTS_KEY));

		ee_forces = pose_task->getSensedForceControlWorldFrame();
		ee_moments = pose_task->getSensedMomentControlWorldFrame();

		if (ee_forces.norm() > 0.0001) {
			cout << ee_forces.transpose() << endl;
		}

		
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if ((ee_pos - ee_pos_desired).norm() < 1e-3) {
				cout << "Posture To Motion Down" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos_desired = ee_pos + Vector3d(0.0, 0.0, -0.2);
				q_desired = robot_q;

				cout << "Desired end-effector position: " << ee_pos_desired.transpose() << endl;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalOrientation(ee_ori);
				joint_task->setGoalPosition(q_desired);

				state = MOTION_DOWN;
			}
		} else if (state == MOTION_DOWN) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if ((ee_pos - ee_pos_desired).norm() < 1e-3) {
				cout << "Motion Down to Motion Up" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos_desired = initial_ee_pos + Vector3d(0.0, 0.0, 0.2);
				q_desired = robot_q;

				cout << "Desired end-effector position: " << ee_pos_desired.transpose() << endl;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalOrientation(ee_ori);
				joint_task->setGoalPosition(q_desired);

				state = MOTION_UP;
			}
		} else if (state == MOTION_UP) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			if ((ee_pos - ee_pos_desired).norm() < 1e-3) {
				cout << "Motion Up to Motion Down" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos_desired = initial_ee_pos + Vector3d(0.0, 0.0, -0.2);
				q_desired = robot_q;

				cout << "Desired end-effector position: " << ee_pos_desired.transpose() << endl;

				pose_task->setGoalPosition(ee_pos_desired);
				pose_task->setGoalOrientation(ee_ori);
				joint_task->setGoalPosition(q_desired);

				state = MOTION_DOWN;
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
