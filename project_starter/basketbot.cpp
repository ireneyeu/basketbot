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
	Ball() : position(Vector3d::Zero()), velocity(Vector3d::Zero()), apex(0.0), contact(0.0f) {}

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
		return position(0) >  0.1 && position(0) < 1.5; 
			// && position(1) > -0.8 && position(1) < 0.8; // Uncomment if checking ball valid y
	}

	Vector3d position;
	Vector3d velocity;
	float apex;	float contact;
};

// EE Class
class EndEffector {
	public:
		EndEffector()
			: pos(Vector3d::Zero()), vel(Vector3d::Zero()), ori(Matrix3d::Identity()),
			pos_init(Vector3d::Zero()), ori_init(Matrix3d::Identity()),
			pos_desired(Vector3d::Zero()), vel_desired(Vector3d::Zero()),
			ori_desired(ori_init), contact(Vector3d::Zero()), lowest (0.0f),
			final_down_goal(Vector3d::Zero()), down_direction(Vector3d::Zero()) {}
	
	void update(std::shared_ptr<SaiModel::SaiModel> robot, const string& link, const Vector3d& point) {
			pos = robot->position(link, point);
			vel = robot->linearVelocity(link, point);
			ori = robot->rotation(link);
		}
	
	void setInitial() {
		pos_init = pos;
		// ori_init = [1 0 0; 0 -1 0; 0 0 -1];
		ori_init = Matrix3d::Identity();
		ori_init(1, 1) = -1.0;
		ori_init(2, 2) = -1.0;
		// ori_init = ori;
	}
	
	// Takes ball position and follows x + offset, y? and z initial
	void trackXY(const Vector3d& ball_pos, float offset_x, bool tracking_x = false, bool tracking_y = false) {
		pos_desired = pos_init;
		if (tracking_x)
			pos_desired(0) = clamp(ball_pos(0) + offset_x, 0.4, 0.75);
		if (tracking_y)
			pos_desired(1) = ball_pos(1);
	}
	
	void wristGoal(const float& theta_deg){
		ori_desired = AngleAxisd(theta_deg*M_PI/180.0, ori_init.col(1)).toRotationMatrix() * ori_init;
	}

	void trackBallWithAngles(const Vector3d& ball_pos, bool track_q1 = true, bool track_q2  = true) {
		// q1 is angle to make up for x error, rotates about ee y
		if (track_q1) {
			float q1 = atan((ball_pos(0) - pos_init(0)) / pos_init(2)) / 2.0;    
			ori_desired = AngleAxisd(q1, -ori_init.col(1)).toRotationMatrix() * ori_desired;
		}

		// q2 is angle to make up for y error, rotates about ee x
		if (track_q2) {
			float q2 = atan((ball_pos(1) - pos_init(1)) / pos_init(2)) / 2.0;
			ori_desired = AngleAxisd(q2, -ori_init.col(0)).toRotationMatrix() * ori_desired;
		}
	}

	void setDownDirection(bool tracking_x = false, bool tracking_y = false) {
		final_down_goal = pos;
		if (tracking_x) {
			final_down_goal(0) = 0.5 * (pos(0) + pos_init(0));
		}
		if (tracking_y) {
			final_down_goal(1) = 0.5 * (pos(1) + pos_init(1));
		}
		final_down_goal(2) = -0.3;
		down_direction = final_down_goal - pos;
		down_direction.normalize();
	}

	bool isValid() const {
		return pos(0) >  0.35 && pos(0) < 0.8
			&& pos(1) > -0.4  && pos(1) < 0.4 
			&& pos(2) >  0.25  && pos(2) < 0.7;
	}

	Vector3d pos, pos_init, pos_desired, vel, vel_desired, contact, final_down_goal, down_direction;
	Matrix3d ori, ori_init, ori_desired;
	float lowest;
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
	joint_task->setGains(40, 10, 0);
	// Positional gains
	Vector3d kp_xyz(200.0, 200.0, 200.0);
	Vector3d kv_xyz(40.0, 40.0, 40.0);
	// Orientation gains
	Vector3d kp_ori_xyz(250.0, 250.0, 250.0);
	Vector3d kv_ori_xyz( 50.0,  50.0,  50.0);

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
	bool simulation       = false;     // true for simulation, false for real robot
	bool tracking_x       = false;	   // for position and angle tracking in x
	bool tracking_y       = false;     // for position and angle tracking in y
	bool tracking_x_angle = false;     // for angle tracking in x
	bool tracking_y_angle = false;     // for angle tracking in y
	bool tracking_apex    = true;     // for tracking ball apex
	bool apex_condition   = true;      // true by default
	bool up_test          = true;     // for tests 1 and 2 hardcoded
	float x_ball_offset   = -0.22;     // offset in x for desired ee point
	double z_ball_offset  = 0.15;      // offset in z for desired ee point
	float wrist_up_deg    = 10.0;      // wrist up goal angle in degrees
	float wrist_down_deg  = -20.0;     // wrist down goal angle in degrees
	float step_size       = 0.008;    // step size for "velocity" control
	float step            = 0.008;    // initial step
	float step_up_size    = 0.001;
	float step_up         = 0.001;
	float ball_vel_up     = 0.1;       // velocity threshold considering the ball going up
	float ball_vel_down   = 0.01;      // velocity threshold considering the ball going down
	float min_ball_apex   = 0.01;       // min ball apex height to consider dribbling
	float clamp_z         = 0.05;      // min and max z position for ee during motion up
	float start_down_dis  = 0.10;      // threshold for down motion
	float stop_down_dis   = 0.18;      // threshold for stopping down motion
	float max_down_dis    = 0.04;      // max distance for down motion
	float prev_contact    = 0.5;
	float waiting_offset  = start_down_dis*1.0;
	bool verbose          = false;
	bool reached_up       = false;
	bool driver_running   = true;
	
	// "1" = hard coded, "2" = hard coded with speed, "3" = test up down with orientation, 
	// "4" = tracking XZ, "5" = orientation incline, "6" = test3 plus tracking the ball and orientation,
	// "7" = waiting
	string controller_status = "7";
	float freq = 8.5; // 8.5 was tried and worked

	// // Define Information
	VectorXd q_desired(7);
	q_desired << 0.0, -0.1, 0.0, -2.0, 0.0, 1.9, -0.77; // sai::sensors::FrankaRobot::joint_positions
	// q_desired << 0.0, 0.05, 0.0, -2.0, 0.0, 2.2, -0.77; // sai::sensors::FrankaRobot::joint_positions
	//q_desired << 0.0, 0.27, 0.0, -2.0, 0.0, 2.3, -0.77; // sai::sensors::FrankaRobot::joint_positions
	
	// initial state 
	int state = POSTURE;
	float dribble_count = 0.0;

	Ball ball;
	EndEffector ee;

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
	} 
	catch (const std::exception& e) {
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
	joint_task->setGoalPosition(q_desired);	

	// create a loop timer
	runloop = true;
	double control_freq = 1000; // should be 1000
	SaiCommon::LoopTimer timer(control_freq, 1e6);
	double time_start;

	cout << "Entering controller loop: STATE[" << state << "]" << endl;

	VectorXd controller_check_prev = redis_client.getEigen("sai::sensors::FrankaRobot::model::robot_gravity");

	int iteration = 0;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// updates
		updateRobotModel(robot, redis_client, simulation, MASS_MATRIX_KEY, JOINT_ANGLES_KEY, JOINT_VELOCITIES_KEY);
		ball.update(redis_client);
		ee.update(robot, control_link, control_point);

		if (iteration % 10 == 0) {
			VectorXd controller_check = redis_client.getEigen("sai::sensors::FrankaRobot::model::robot_gravity");
			if (controller_check == controller_check_prev && driver_running){
				cout << "Controller Dead" << endl;
				driver_running = false;
				state = STOP;
			}
			controller_check_prev = controller_check;
			iteration = 1;
		}
		iteration ++;
		

		if (controller_status == "7" && state != STOP) {
			if (!ball.isValid()) {
				cout << "STOPPED BALL: " << ball.position.transpose() << endl;
				state = STOP;
			}
		}
		if (!ee.isValid() && state != STOP) {
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
			cout << "Robot Joints norm: " << (robot->q() - q_desired).norm() << "->  " << (robot->q() - q_desired).transpose()<< endl;

			if ((robot->q() - q_desired).norm() < 15e-2) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee.setInitial();
				ee.contact = ee.pos;
				ball.contact = ee.pos(2) - z_ball_offset - waiting_offset;
				ee.lowest = ee.contact(2);
				prev_contact = ball.contact;
				cout << "EE Initial Orientation: " << ee.ori << endl;
				cout << "EE Initial Position: " << ee.pos.transpose() << endl;

				pose_task->setGoalPosition(ee.pos);
				time_start = time;

				if (controller_status == "1") {
					cout << "TEST1: HardCoded" << endl;
					cout << ee.pos.transpose() << endl;
					state = TEST1;
				} else if (controller_status == "2") {
					cout << "TEST2: HardCoded w. velocity" << endl;
					state = TEST2;
				} else if (controller_status == "3") {
					cout << "TEST3: Up-Down with orientation sines" << endl;
					state = TEST3;
				} else if (controller_status == "4") {
					if (ball.isValid() && ee.isValid()){
						cout << "TEST4: Calibrating X-Z offsets" << endl;
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
			ee.trackXY(ball.position, x_ball_offset, tracking_x, tracking_y); // (ball.position(x,y,z), offset_x, bool tracking_x, bool tracking_y)


			if (ee.pos(2) < ball.contact + z_ball_offset + waiting_offset && !reached_up){ // 0.27 < 0.267 + 0.15 + 0.20 = 0.61
				//cout << "velcontrol happening" << endl;
				ee.pos_desired(2) = ee.lowest + step_up;
				step_up += step_up_size;
			} else {
				ee.pos_desired(2) = ball.contact + z_ball_offset + waiting_offset;
				//cout << "reached up" << endl;
				reached_up = true;
			}

			// if ((ee.pos-ee.pos_desired).norm()< 0.01){
			// 	cout << "Reached Waiting Position Desired" << endl;
			// }

			// ee.pos_desired(2) = ball.contact + z_ball_offset + .04;

			pose_task->setGoalPosition(ee.pos_desired);
			
			// orientation goals
			ee.wristGoal(wrist_up_deg);
			ee.trackBallWithAngles(ball.position, tracking_x_angle, tracking_y_angle); // Updates ee.ori_desired considering ball position x error w/ q1 and y error w/q2
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (verbose){
				cout << "WAITING COMMAND: " << ee.pos(2) << ", desired: "<< ee.pos_desired(2) << "ball contact: " << ball.contact << endl;
			}

			if (tracking_apex){
				apex_condition = ball.apex > min_ball_apex;
			}


			if (ball.isValid() && ball.velocity(2) > ball_vel_up && ee.pos(2) - (ball.position(2) + z_ball_offset) > 0.1 && apex_condition) {
				state = MOTION_UP;
				step_up = step_up_size;
				reached_up = false;
				cout << "WAITING TO MOVING UP. POS: " << ee.pos(2) << " Desired: " << ee.pos_desired(2) << endl << endl;
			}
		} else if (state == MOTION_UP) {
			// position goals 
			ee.trackXY(ball.position, x_ball_offset, tracking_x, tracking_y); // (ball.position(x,y,z), offset_x, bool tracking_x, bool tracking_y)
			if (tracking_apex) {
				ee.pos_desired(2) = clamp(ball.apex + z_ball_offset, ee.contact(2) - clamp_z, ee.contact(2) + clamp_z); //(value, min, max)
			} else {
				ee.pos_desired(2) = ball.position(2) + z_ball_offset;
			}

			if (verbose){
				cout << "UP COMMAND: " << ee.pos(2) << ", desired: "<< ee.pos_desired(2) << endl;
			}
			pose_task->setGoalPosition(ee.pos_desired);

			// orientation goals           
			ee.wristGoal(wrist_up_deg);
			ee.trackBallWithAngles(ball.position, tracking_x_angle, tracking_y); // Updates ee.ori_desired considering ball position x error w/ q1 and y error w/q2
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (ball.velocity(2) < ball_vel_down || ee.pos(2) - (ball.position(2) + z_ball_offset) < start_down_dis ) {
				state = MOTION_DOWN;
				if (ball.velocity(2) < ball_vel_down) {
					cout << " MOTION UP TO MOTION DOWN. POS: " << ee.pos(2) << " Desired: " << ee.pos_desired(2)<< endl;
					cout << " Caused by: Negative ball velocity." << endl << endl;
				} else {
					cout << " MOTION UP TO MOTION DOWN. POS: " << ee.pos(2) << " Desired: " << ee.pos_desired(2) << endl;
					cout << " Caused by: Ball is close enough. " << endl << endl;
				}

				ee.contact = ee.pos;
				ball.contact = ball.position(2);
				if (prev_contact - ball.contact > .02) {
					ball.contact = prev_contact - .02;
				}
				cout << "CONTACT: " << ball.contact << endl;
				prev_contact = ball.contact;

				// velocity control
				ee.setDownDirection(tracking_x, tracking_y);

				ee.wristGoal(wrist_down_deg);
				pose_task->setGoalOrientation(ee.ori_desired);
				}	
				
		} else if (state == MOTION_DOWN) {
			// update task model

			// velocity control
			ee.pos_desired = ee.contact + step*ee.down_direction;

			if (step < max_down_dis + 0.02){
				step += step_size;
			}
			if (verbose){
				cout << "DOWN COMMAND: " << ee.pos(2) << ", desired: "<< ee.pos_desired(2) << endl;
			}
			pose_task->setGoalPosition(ee.pos_desired);

			
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

			if (ee.pos(2) - (ball.position(2) + z_ball_offset) > stop_down_dis || ee.contact(2) - ee.pos(2) > max_down_dis) {
				state = WAITING;
				if (ee.pos(2) - (ball.position(2) + z_ball_offset) > stop_down_dis){
					cout << " MOTION DOWN TO WAITING. POS: " << ee.pos(2) << " Desired: " << ee.pos_desired(2) << endl;
					cout << " Caused by: Ball is far enough. " << endl << endl;
				} else {
					cout << " MOTION DOWN TO WAITING POS: " << ee.pos(2) << " Desired: " << ee.pos_desired(2) << endl;
					cout << " Caused by: EE moved enough in z" << endl << endl;
				}
				ee.lowest = ee.pos(2);
				dribble_count += 1.0;
				step = step_size;
			}
		} else if (state == TEST1) {
			ee.pos_desired = ee.pos_init;
			ee.ori_desired = ee.ori_init;
			if (up_test){
				ee.pos_desired(2) = ee.pos_init(2) + 0.03;
				ee.wristGoal(wrist_up_deg);
			} else {
				ee.pos_desired(2) = ee.pos_init(2) - 0.03;
				ee.wristGoal(wrist_down_deg);
			}
			float diff_pos = abs(ee.pos(2) - ee.pos_desired(2));
			float diff_ori = (ee.ori - ee.ori_desired).norm();

			cout << diff_ori << endl;
			if (diff_ori < 0.05){
				up_test = !up_test;
				cout << "SWITCHING!!!" << endl;
			}
			pose_task->setGoalPosition(ee.pos_desired);
			pose_task->setGoalOrientation(ee.ori_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST2) {
			ee.pos_desired = ee.pos_init;
			ee.ori_desired = ee.ori_init;
			
			Vector3d final_pos_up;
			final_pos_up << ee.pos_init(0), ee.pos_init(1), ee.pos_init(2) + 0.5;
			Vector3d direction = final_pos_up - ee.pos;
			direction.normalize();

			if (up_test){
				ee.pos_desired = ee.pos + step*direction;
				ee.wristGoal(wrist_up_deg);
			} else {
				ee.pos_desired = ee.pos - step*direction;
				ee.wristGoal(wrist_down_deg);
			}
			float diff_pos = ee.pos(2) - ee.pos_init(2);
			float diff_ori = (ee.ori - ee.ori_desired).norm();

			// step += step_size;

			// cout << diff_ori << endl;
			if (up_test && diff_pos > 0.03){
				up_test = !up_test;
				step = step_size;
				cout << "SWITCHING!!!" << endl;
			}
			if (!up_test && diff_pos < 0.03){
				up_test = !up_test;
				step = step_size;
				cout << "SWITCHING!!!" << endl;
			}
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
			ee.pos_desired = ee.pos_init;
			ee.trackXY(ball.position, x_ball_offset, tracking_y);
			ee.pos_desired(2) = ball.position(2) + z_ball_offset;

			pose_task->setGoalPosition(ee.pos_desired);

			// update task model
			command_torques = updateCommandTorques(*pose_task, *joint_task, N_prec);

		} else if (state == TEST5) {
			ee.pos_desired = ee.pos_init;
			ee.trackBallWithAngles(ball.position, tracking_x_angle, tracking_y);
			
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