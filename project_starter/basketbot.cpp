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


// State Machine States
enum State {
	POSTURE = 0, WAITING, MOTION_UP, MOTION_DOWN,
	TEST1, TEST2, TEST3, TEST4, TEST5, TEST6,
	STOP
};

// Ball Class
class Ball {
public:
	Ball() : position(Vector3d::Zero()), velocity(Vector3d::Zero()), apex(0.0) {}

	void update(SaiCommon::RedisClient &redis) {
		position = redis.getEigen(BALL_POSITION_KEY);
		velocity = redis.getEigen(BALL_VELOCITY_KEY);
		apex = std::stof(redis.get(BALL_APEX_KEY));
	}

	void pushToRedis(SaiCommon::RedisClient &redis) const {
		redis.setEigen(BALL_POSITION_KEY, position);
		redis.setEigen(BALL_VELOCITY_KEY, velocity);
	}

	bool isValid() const {
		return position(0) >  0.1 && position(0) < 1.2; 
			// && position(1) > -0.8 && position(1) < 0.8; // Uncomment if checking ball valid y
	}

	Vector3d position;
	Vector3d velocity;
	float apex;
};

// EE Class
class EndEffector {
	public:
		EndEffector(const Matrix3d& ori_init)
			: pos(Vector3d::Zero()), vel(Vector3d::Zero()), ori(Matrix3d::Identity()),
			pos_init(Vector3d::Zero()), ori_init(ori_init),
			pos_desired(Vector3d::Zero()), vel_desired(Vector3d::Zero()),
			ori_desired(ori_init), contact(0.0f) {}
	
	void update(std::shared_ptr<SaiModel::SaiModel> robot, const string& link, const Vector3d& point) {
			pos = robot->position(link, point);
			vel = robot->linearVelocity(link, point);
			ori = robot->rotation(link);
		}
	
		void setInitial() {
			pos_init = pos;
			ori_init = ori;
		}
	
		// Takes ball position and follows x + offset, y? and z initial
		void trackXY(const Vector3d& ball_pos, float offset_x = -0.20, bool tracking_y = false) {
			pos_desired = pos_init;
			pos_desired(0) = ball_pos(0) + offset_x;
			if (tracking_y)
				pos_desired(1) = ball_pos(1);
		}
	
		void wristGoal(const float& theta_deg){
			ori_desired = AngleAxisd(theta_deg*M_PI/180.0, ori_init.col(1)).toRotationMatrix() * ori_init;
		}

		void trackBallWithAngles(const Vector3d& ball_pos, bool q1 = true, bool q2  = true) {
			// q1 is angle to make up for x error, rotates about ee y
			if (q1) {
				float q1 = atan((ball_pos(0) - pos_init(0)) / pos_init(2)) / 2.0;    
				ori_desired = AngleAxisd(q1, -ori_init.col(1)).toRotationMatrix() * ori_desired;
			}

			// q2 is angle to make up for y error, rotates about ee x
			if (q2) {
				float q2 = atan((ball_pos(1) - pos_init(1)) / pos_init(2)) / 2.0;
				ori_desired = AngleAxisd(q2, -ori_init.col(0)).toRotationMatrix() * ori_desired;
			}
		}

		bool isValid() const {
			return pos(0) >  0.35 && pos(0) < 0.7 
				&& pos(1) > -0.6  && pos(1) < 0.6 
				&& pos(2) >  0.2  && pos(2) < 0.8;
		}
	
		Vector3d pos, pos_init, pos_desired, vel, vel_desired;
		Matrix3d ori, ori_init, ori_desired;
		float contact;
	};

// Setup KEYS
void setupSaiKeys(bool simulation) {
	if (simulation) {
		cout << "SIMULATION TRUE" << endl;
		JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
		BALL_POSITION_KEY = "sai::sim::BALL::sensors::position";
		BALL_VELOCITY_KEY = "sai::sim::BALL::sensors::velocity";
	} else {
		cout << "SIMULATION FALSE" << endl;
		JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";
		JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
		JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
		BALL_POSITION_KEY = "sai::camera::BALL::sensors::position";
		BALL_VELOCITY_KEY = "sai::camera::BALL::sensors::velocity";
		MASS_MATRIX_KEY = "sai::sensors::FrankaRobot::model::mass_matrix";
	}
}

// Setup Gains
void setupGains(
	std::shared_ptr<SaiPrimitives::MotionForceTask>& pose_task,
	std::shared_ptr<SaiPrimitives::JointTask>& joint_task)
{
	// // Joint-level gains: kp, kv, ki
	// joint_task->setGains(50, 14, 0);
	// // Positional gains
	// Vector3d kp_xyz(100.0, 100.0, 100.0);
	// Vector3d kv_xyz( 20.0,  20.0,  20.0);
	// // Orientation gains
	// Vector3d kp_ori_xyz(200.0, 200.0, 200.0);
	// Vector3d kv_ori_xyz( 20.0,  20.0,  20.0);

	// Joint-level gains: kp, kv, ki
	joint_task->setGains(25, 7, 0);
	// Positional gains
	Vector3d kp_xyz(50.0, 50.0, 50.0);
	Vector3d kv_xyz(5.0, 5.0, 5.0);
	// Orientation gains
	Vector3d kp_ori_xyz(50.0, 50.0, 50.0);
	Vector3d kv_ori_xyz( 5.0,  5.0,  5.0);

	pose_task->setPosControlGains(kp_xyz, kv_xyz);
	pose_task->setOriControlGains(kp_ori_xyz, kv_ori_xyz);    
}

// Update robot model
void updateRobotModel(shared_ptr<SaiModel::SaiModel> robot,
	SaiCommon::RedisClient &redis,
	bool simulation,
	const string &mass_matrix_key,
	const string &joint_angles_key,
	const string &joint_velocities_key) 
{
	robot->setQ(redis.getEigen(joint_angles_key));
	robot->setDq(redis.getEigen(joint_velocities_key));

	MatrixXd M = robot->M();
	if (!simulation) {
	M = redis.getEigen(mass_matrix_key);
	M(4,4) += 0.2;
	M(5,5) += 0.2;
	M(6,6) += 0.2;
	}
	robot->updateModel(M);
}

// Update command torques
Eigen::VectorXd updateCommandTorques(
	SaiPrimitives::MotionForceTask& pose_task,
	SaiPrimitives::JointTask& joint_task,
	Eigen::MatrixXd& N_prec
) {
	N_prec.setIdentity();  // Reset nullspace projection
	pose_task.updateTaskModel(N_prec);
	joint_task.updateTaskModel(pose_task.getTaskAndPreviousNullspace());
	return pose_task.computeTorques() + joint_task.computeTorques();
}


int main() {
	bool simulation = true;
	bool tracking_y = false;

	// "1" = test up down, "2" = test orientation speed, "3" = test up down with orientation, 
	// "4" = test3 plus tracking the ball, "5" = orientation incline, "6" = test3 plus tracking the ball and orientation,
	// "7" = waiting
	string controller_status = "7";
	float freq = 8.5; // 8.5 was tried and worked

	// // Define Information
	VectorXd q_desired(7);
	q_desired << -0.0389406,-0.25928,-0.0295874,-2.13659,-0.00835724,1.91865,-0.759372;


	// initial state 
	int state = POSTURE;
	float dribble_count = 0.0;

	Ball ball;
	EndEffector ee(Matrix3d::Identity());

	setupSaiKeys(simulation);

	// Location of URDF files specifying world and robot information
	static const string robot_file = string(BASKETBOT_URDF_FOLDER) + "/panda/panda_arm_box.urdf";

	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);


	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);

	// Try reading Redis keys; fallback to defaults if not present (in order to launch controller first)
	try {
		VectorXd q = redis_client.getEigen(JOINT_ANGLES_KEY);
		VectorXd dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
		ball.update(redis_client);
		robot->setQ(q);
		robot->setDq(dq);
	} catch (const std::exception& e) {
		std::cerr << "Warning: redis values empty." << "\nSetting default joint values in Redis." << std::endl;
		VectorXd q = VectorXd::Zero(7);
		VectorXd dq = VectorXd::Zero(7);
		robot->setQ(q);
		robot->setDq(dq);
		ball.position << 0.0, 0.575, 0.0;
		ball.velocity << 0.0, 0.0, 3.0;		
		//redis_client.setEigen(JOINT_ANGLES_KEY, robot_q_init);
		//redis_client.setEigen(JOINT_VELOCITIES_KEY, robot_dq_init);
		ball.pushToRedis(redis_client);
	}

	MatrixXd M = robot->M();
	if(!simulation) {
		M = redis_client.getEigen(MASS_MATRIX_KEY);
		// bie addition
		M(4,4) += 0.2;
		M(5,5) += 0.2;
		M(6,6) += 0.2;
	}
	robot->updateModel(M);

	// prepare controllers
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0.0, 0.0, 0.0);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	
	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);

	// Desired robot states and gains
	setupGains(pose_task, joint_task);

	// Set tasks
	pose_task->disableInternalOtg();
	pose_task->setGoalPosition(ee.pos_desired);
	joint_task->setGoalPosition(q_desired);	

	// create a loop timer
	runloop = true;
	double control_freq = 1000; // should be 1000
	SaiCommon::LoopTimer timer(control_freq, 1e6);
	double time_start;

	cout << "Entering controller loop" << endl;
	cout << "["<< state << "]" << endl;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// updates
		updateRobotModel(robot, redis_client, simulation, MASS_MATRIX_KEY, JOINT_ANGLES_KEY, JOINT_VELOCITIES_KEY);
		ball.update(redis_client);
		ee.update(robot, control_link, control_point);

		// Checking if ball is valid: in range and not anomalies
		if (!ball.isValid()) {
			cout << "STOPPED BALL: " << ball.position.transpose() << endl;
			state = STOP;
		}
		if (!ee.isValid()) {
			cout << "STOPPED EE: " << ee.pos.transpose() << endl;
			state = STOP;
		}

		if (state == STOP){
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();
		}

		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			command_torques = joint_task->computeTorques();
			cout << "Robot Joints norm: " << (robot->q() - q_desired).norm() << endl;

			if ((robot->q() - q_desired).norm() < 25e-2) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee.setInitial();
				ee.contact = ee.pos(2);

				pose_task->setGoalPosition(ee.pos);
				time_start = time;

				if (controller_status == "1") {
					cout << "TEST1: Up-Down sine" << endl;
					state = TEST1;
				} else if (controller_status == "2") {
					cout << "TEST2: Orientation up-down sine" << endl;
					state = TEST2;
				} else if (controller_status == "3") {
					cout << "TEST3: Up-Down with orientation sines" << endl;
					state = TEST3;
				} else if (controller_status == "4") {
					cout << "TEST4: Test3 + following ball" << endl;
					if (ball.isValid() && ee.isValid()){
						cout << "TEST4: Following ball in waiting" << endl;
						state = TEST4;
					} else if (!ee.isValid()){
						cout << "BAD EE: " << ee.pos.transpose() << endl;
					} else if (!ball.isValid()){
						cout << "BAD BALL: " << ball.position.transpose() << endl;
					}
				} else if (controller_status == "5") {
					cout << "TEST5: orientation following" << endl;
					state = TEST5;
				} else if (controller_status == "6") {
					cout << "TEST6: Test3 + following ball and orientation" << endl;
					cout << ball.position.transpose() << endl;
					state = TEST6;
				} else if (controller_status == "7") {
					if (ball.isValid() && ee.isValid()){
						cout << "TEST7: Following ball in waiting" << endl;
						state = WAITING;
					} else if (!ee.isValid()){
						cout << "BAD EE: " << ee.pos.transpose() << endl;
					} else if (!ball.isValid()){
						cout << "BAD BALL: " << ball.position.transpose() << endl;
					}
				}
			}
		}

		if (state == WAITING) {
			// update task model 
			ee.trackXY(ball.position, -0.20, tracking_y); // (ball.position(x,y,z), offset_x, bool tracking_y)
			ee.pos_desired(2) = ee.contact;
			pose_task->setGoalPosition(ee.pos_desired);

			// orientation goals
			float wrist_up_deg = 20.0;

			ee.wristGoal(wrist_up_deg);
			ee.trackBallWithAngles(ball.position, true, tracking_y); // Updates ee.ori_desired considering ball position x error w/ q1 and y error w/q2
			pose_task->setGoalOrientation(ee.ori_desired);

			// ee.vel_desired(0) = ball.velocity(0);
			// ee.vel_desired(1) = ball.velocity(1);
			// pose_task->setGoalLinearVelocity(ee.vel_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (ball.isValid() && ball.velocity(2) > 0.1 && ball.apex > 0.2) {
				cout << "Ball Going up" << endl;
				cout << "WAITING TO MOVING UP" << endl;

				state = MOTION_UP;
				cout << "["<< state << "]" << endl;
			}
		} else if (state == MOTION_UP) {
			// position goals 
			ee.trackXY(ball.position, -0.20, tracking_y); // (ball.position(x,y,z), offset_x, bool tracking_y)
			ee.pos_desired(2) = clamp(ball.apex + 0.20, ee.pos_init(2) - 0.15, ee.pos_init(2) + 0.15); //(value, min, max)

			pose_task->setGoalPosition(ee.pos_desired);

			// orientation goals
			float wrist_up_deg = 10.0;            
			ee.wristGoal(wrist_up_deg);
			ee.trackBallWithAngles(ball.position, true, tracking_y); // Updates ee.ori_desired considering ball position x error w/ q1 and y error w/q2
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (ball.velocity(2) < .1) {
				cout << "MOTION UP TO MOTION DOWN" << endl;

				ee.contact = ee.pos(2);

				ee.pos_desired(0) = 0.5* (ee.pos(0) + ee.pos_init(0));
				ee.pos_desired(1) = 0.5* (ee.pos(1) + ee.pos_init(1));
				ee.pos_desired(2) = -0.2;
				pose_task->setGoalPosition(ee.pos_desired);

				float wrist_down_deg = -30.0;
				ee.wristGoal(wrist_down_deg);
				pose_task->setGoalOrientation(ee.ori_desired);
			
				state = MOTION_DOWN;
				cout << "["<< state << "]" << endl;
			}	
				
		} else if (state == MOTION_DOWN) {
			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (abs(ball.position(2) - ee.pos(2)) > 0.30 || ee.pos(2) < ee.contact-0.03 ) {
				cout << "MOTION DOWN TO WAITING" << endl;
				dribble_count += 1.0;

				state = WAITING;
				cout << "["<< state << "]" << endl;
			}
		} else if (state == TEST1) {
			ee.pos_desired = ee.pos_init;
			ee.pos_desired(2) = ee.pos_init(2) + 0.1 * sin(freq*(time - time_start));

			ee.vel_desired(0) = 0;
			ee.vel_desired(1) = 0;
			ee.vel_desired(2) = 0.1*freq*cos(freq*(time - time_start));

			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalLinearVelocity(ee.vel_desired);
			
			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST2) {
			float theta = -3*M_PI/180.0 + 20.0*M_PI/180.0 * sin(freq*(time - time_start));
			ee.pos_desired = ee.pos_init;

			ee.ori_desired = AngleAxisd(theta, ee.ori_init.col(1)).toRotationMatrix() * ee.ori_init;
			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST3) {
			ee.pos_desired = ee.pos_init;
			ee.pos_desired(2) = ee.pos_init(2) + 0.03 * sin(freq*(time - time_start));

			ee.vel_desired(0) = 0;
			ee.vel_desired(1) = 0;
			ee.vel_desired(2) = 0.03*freq*cos(freq*(time - time_start));

			float theta = 5.0*M_PI/180.0 + 25.0*M_PI/180.0 * sin(freq*(time - time_start));
			ee.ori_desired = AngleAxisd(theta, ee.ori_init.col(1)).toRotationMatrix() * ee.ori_init;

			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalOrientation(ee.ori_desired);
			pose_task->setGoalLinearVelocity(ee.vel_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST4) {
			ee.trackXY(ball.position, -0.20, tracking_y); // (ball.position(x,y,z), offset_x, bool tracking_y)
			ee.pos_desired(2) = ball.position(2) + 0.35;
			// ee.pos_desired(2) = ee.pos_init(2) + 0.02 * sin(freq*(time - time_start));

			// ee.vel_desired(0) = ball.velocity(0);
			// ee.vel_desired(1) = ball.velocity(1);
			// ee.vel_desired(2) = 0.0;

			float theta = -3*M_PI/180.0 + 20.0*M_PI/180.0 * sin(freq*(time - time_start));
			// ee.ori_desired = AngleAxisd(theta, ee.ori_init.col(1)).toRotationMatrix() * ee.ori_init;

			pose_task->setGoalPosition(ee.pos_desired);
			// pose_task->setGoalOrientation(ee.ori_desired);
			// pose_task->setGoalLinearVelocity(ee.vel_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST5) {
			ee.pos_desired = ee.pos_init;
			ee.trackBallWithAngles(ball.position, true, tracking_y);
			
			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST6) {
			// Tracking ball in x-y and sine in z
			ee.trackXY(ball.position, -0.20, tracking_y);
			// ee.pos_desired(2) = ee.pos_init(2) + 0.02 * sin(freq*(time - time_start));
			// velocitiy
			ee.vel_desired(0) = 0;
			ee.vel_desired(1) = 0;
			ee.vel_desired(2) = 0;
			// ee.vel_desired(2) = 0.02*freq*cos(freq*(time - time_start));

			// Sine in orientation
			float theta = -3*M_PI/180.0 + 20.0*M_PI/180.0 * sin(freq*(time - time_start));
			// ee.ori_desired = AngleAxisd(theta, ee.ori_init.col(1)).toRotationMatrix() * ee.ori_init;
			ee.trackBallWithAngles(ball.position, true, tracking_y);

			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalOrientation(ee.ori_desired);
			pose_task->setGoalLinearVelocity(ee.vel_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigen(EE_POSITION_KEY, ee.pos);
		redis_client.setEigen(EE_VELOCITY_KEY, ee.vel);
	}

	timer.stop();
	cout << "Total robot dribble count: " << dribble_count << endl;
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}