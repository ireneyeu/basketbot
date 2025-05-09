/**
 * @file basketbot-simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string robot_name = "PANDA";
static const string camera_name = "camera_fixed";

// dynamic objects information
const vector<std::string> object_names = {"BALL"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// Force sensor information
const string link_name = "end-effector";
const Vector3d control_point = Vector3d(0.2, 0, 0.12);
Affine3d compliant_frame = Affine3d::Identity();
Vector3d sensed_force;
Vector3d sensed_moment;
double cutoff_freq = 2.0;

// Bal Information
Vector3d ball_position;
Vector3d ball_velocity;
Vector3d ball_spin;
Affine3d ball_pose = Affine3d::Identity();

// simulation thread
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

int main() {
	SaiModel::URDF_FOLDERS["BASKETBOT_URDF_FOLDER"] = std::string(BASKETBOT_URDF_FOLDER);
	SaiModel::URDF_FOLDERS["BASKETBOT_FOLDER"] = std::string(PROJECT_STARTER);
	static const string robot_file = string(BASKETBOT_URDF_FOLDER) + "/panda/panda_arm_box.urdf";
	static const string world_file = string(PROJECT_STARTER) + "/world.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background
	graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	// robot->setQ();
	// robot->setDq();
	robot->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = std::make_shared<SaiSimulation::SaiSimulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

	// set ball information
	ball_position<< 0.65, 0.0, 0.0;
	ball_velocity << 0.0, 0.0, 2.8;
	ball_spin << 0.0, 0.0, 0.0;
	ball_pose = Affine3d::Identity();
	ball_pose.translation() = ball_position;

	sim->setObjectPose("BALL", ball_pose);
	sim->setObjectVelocity("BALL", ball_velocity);

	// set co-efficient of restition to zero for force control
	sim->setCollisionRestitution(0.0);
	sim->setCollisionRestitution(1.0, "BALL");
	sim->setCollisionRestitution(1.0, "Floor");

	// set co-efficient of friction
	sim->setCoeffFrictionStatic(0.0);
	sim->setCoeffFrictionDynamic(0.0);

	// set up force sensor
	sim->addSimulatedForceSensor(robot_name, link_name, compliant_frame, cutoff_freq);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());
	redis_client.setEigen(EE_FORCES_KEY, Vector3d::Zero());
	redis_client.setEigen(EE_MOMENTS_KEY, Vector3d::Zero());
	redis_client.setEigen(BALL_POSITION_KEY, ball_position);
	redis_client.setEigen(BALL_VELOCITY_KEY, ball_velocity);

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
		graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim) {
	// fSimulationRunning = true;

	// create redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// create a timer
	double sim_freq = 2000; // should be 2000
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq); // 0.1 is 10 times slower sim, 1.0 is real time
	sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
		redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
		redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
			ball_position = sim->getObjectPose("BALL").translation();
			ball_velocity = sim->getObjectVelocity("BALL").head(3);
			ball_spin = sim->getObjectVelocity("BALL").tail(3);
		}

		// force sensor
		sensed_force = sim->getSensedForce(robot_name, link_name);
		sensed_moment = sim->getSensedMoment(robot_name, link_name);
		redis_client.setEigen(EE_FORCES_KEY, sensed_force);
		redis_client.setEigen(EE_MOMENTS_KEY, sensed_moment);

		// ball
		redis_client.setEigen(BALL_POSITION_KEY, ball_position);
		redis_client.setEigen(BALL_VELOCITY_KEY, ball_velocity);
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
