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
	MOTION
};

int main() {
	// Location of URDF files specifying world and robot information
	// static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_hand.urdf";
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
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	// Robot states
	Vector3d ee_pos;
	Vector3d ee_vel;
	Matrix3d ee_ori;
	VectorXd robot_q(dof);
	VectorXd robot_dq(dof);

	// Desired states
	Vector3d ee_pos_desired;
	Vector3d ee_vel_desired;
	Matrix3d ee_ori_desired;
	VectorXd q_desired(dof);


	//

	// // gripper partial joint task 
	// MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	// gripper_selection_matrix(0, 7) = 1;
	// gripper_selection_matrix(1, 8) = 1;
	// auto gripper_task = std::make_shared<SaiPrimitives::JointTask>(robot, gripper_selection_matrix);
	// gripper_task->setDynamicDecouplingType(SaiPrimitives::DynamicDecouplingType::IMPEDANCE);
	// double kp_gripper = 5e3;
	// double kv_gripper = 1e2;
	// gripper_task->setGains(kp_gripper, kv_gripper, 0);
	// gripper_task->setGains(kp_gripper, kv_gripper, 0);

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	// VectorXd q_desired(dof);
	// q_desired.head(7) << 0.0, 30.0, 0.0, -60.0, 0.0, 90.0, 45.0;
	// q_desired.head(7) *= M_PI / 180.0;
	// // q_desired.tail(2) << 0.04, -0.04;
	// joint_task->setGoalPosition(q_desired);

	
	// Initial robot state
	ee_pos = robot->position(control_link, control_point);
	ee_pos_desired  << 0.3, 0.3, 0.5;
	pose_task->setGoalPosition(ee_pos_desired);
	

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
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

		Vector3d ee_forces = redis_client.getEigen(EE_FORCES);
		Vector3d ee_moments = redis_client.getEigen(EE_MOMENTS);

		if ((ee_forces).norm() > 0.0000001) {
			cout << "EE_FORCES" << redis_client.getEigen(EE_FORCES).transpose() << endl;
			cout << "EE_MOMENTS" << redis_client.getEigen(EE_MOMENTS).transpose() << endl;
		}

		
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			// joint_task->updateTaskModel(N_prec);
			pose_task->updateTaskModel(N_prec);

			// command_torques = joint_task->computeTorques();
			command_torques = pose_task->computeTorques();

			if ((ee_pos - ee_pos_desired).norm() < 1e-3) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				// gripper_task->reInitializeTask();
				joint_task->reInitializeTask();

				pose_task->setGoalPosition(ee_pos + Vector3d(0.0, 0.0, 0.2));

				// pose_task->setGoalOrientation(AngleAxisd(M_PI/4, Vector3d::UnitY()).toRotationMatrix() * ee_ori);

				// gripper_task->setGoalPosition(Vector2d(0.02, -0.02));

				state = MOTION;
			}
		} else if (state == MOTION) {
			// update goal position and orientation
			if (ee_pos(2) > 0.01) {
				ee_pos_desired(2) -= 0.01;
			}
			else if (ee_pos(2) < 0.01) {
				state = POSTURE;
				ee_pos_desired  << 0.4, 0.0, 0.5;
				cout << "Motion To Posture" << endl;
			}

			
			pose_task->setGoalPosition(ee_pos_desired);

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			// gripper_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			// joint_task->updateTaskModel(gripper_task->getTaskAndPreviousNullspace());
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());


			// command_torques = pose_task->computeTorques() + gripper_task->computeTorques() + joint_task->computeTorques();
			command_torques = pose_task->computeTorques() + joint_task->computeTorques();
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
