// some standard library includes
#include <math.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// sai main libraries includes
#include "SaiModel.h"

// sai utilities from sai-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${BASKETBOT_URDF_FOLDER}/panda/panda_arm_box.urdf";

int main(int argc, char** argv) {
    SaiModel::URDF_FOLDERS["BASKETBOT_URDF_FOLDER"] = string(BASKETBOT_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{basketbot} {task_number}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new SaiModel::SaiModel(robot_file);

    // prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);

    const string TIME_KEY = "sai::time"; //Time key for redis
    const string EE_X_KEY = "sai::ee_x"; //End effector position key for redis
    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************
        redis_client.set(TIME_KEY, to_string(time)); // Send time to redis
        redis_client.setEigen(EE_X_KEY, robot->position(link_name, pos_in_link).transpose()); // Send end effector position to redis

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {
            VectorXd q_desired (dof);
            q_desired << robot_q;
            q_desired(6) = 0.1;
            MatrixXd Kp = 400.0*MatrixXd::Identity(dof,dof);
            Kp(dof - 1, dof - 1) = 50.0;
            MatrixXd Kv = 50.0*MatrixXd::Identity(dof,dof);
            Kv(dof - 1, dof - 1) = -0.167;// if  simviz uses 0.1 Kv7 = -0.168; simviz 1.0 -> Kv7 = -0.09


            cout<< robot_q(6) << endl;
            // control_torques.setZero();
            control_torques =  (-Kp * (robot_q - q_desired) - Kv * (robot_dq)) + robot->coriolisForce() + robot->jointGravityVector();
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {
            Vector3d ee_x;
            ee_x = robot->position(link_name, pos_in_link);
            Vector3d ee_xdesired;
            ee_xdesired << 0.3, 0.1, 0.5;

            Vector3d ee_v;
            ee_v = robot->linearVelocity(link_name, pos_in_link);

            Jv = robot->Jv(link_name, pos_in_link);
        	Lambda = robot->taskInertiaMatrix(Jv);

            MatrixXd F = MatrixXd::Zero(3,1);
            float kp = 200.0;
            float kv = 27.0; // 50.0 is overdamped, 5 is underdamped
            F = Lambda*(-kp*(ee_x - ee_xdesired) - kv*(ee_v));
            
            cout << ee_x.transpose() << endl;

            // control_torques.setZero();
            // Part A
            control_torques = Jv.transpose()*F + robot->jointGravityVector(); 

            // Part C
            MatrixXd Kv = 25.0 * MatrixXd::Identity(dof,dof);
            control_torques = control_torques - Kv * (robot_dq); 

            // Part D
            N = robot->nullspaceMatrix(Jv);
            Kv = 25.0* MatrixXd::Identity(dof,dof);
            control_torques = Jv.transpose()*F + robot->jointGravityVector() - N.transpose() * robot->M() * (Kv * (robot_dq));
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            Vector3d ee_x;
            ee_x = robot->position(link_name, pos_in_link);
            Vector3d ee_xdesired;
            ee_xdesired << 0.3, 0.1, 0.5;

            Vector3d ee_v;
            ee_v = robot->linearVelocity(link_name, pos_in_link);

            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);

            Vector3d p;
            p = J_bar.transpose() * robot->jointGravityVector();

            MatrixXd F = MatrixXd::Zero(3,1);
            float kp = 200.0;
            float kv = 27.0; 
            F = Lambda*(-kp*(ee_x - ee_xdesired) - kv*(ee_v)) + p;

            N = robot->nullspaceMatrix(Jv);
            MatrixXd Kv = 25.0* MatrixXd::Identity(dof,dof);
            control_torques = Jv.transpose()*F - N.transpose() * robot->M() * (Kv * (robot_dq));


            cout << ee_x.transpose() << endl;
            // control_torques.setZero();
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {
            Vector3d ee_x;
            ee_x = robot->position(link_name, pos_in_link);
            Vector3d ee_xdesired;
            ee_xdesired << 0.3 + 0.1 * sin(M_PI * time), 0.1 + 0.1 * cos(M_PI * time), 0.5;

            Vector3d ee_v;
            ee_v = robot->linearVelocity(link_name, pos_in_link);

            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);

            Vector3d p;
            p = J_bar.transpose() * robot->jointGravityVector();

            MatrixXd F = MatrixXd::Zero(3,1);
            float kp = 200.0;
            float kv = 27.0; 
            F = Lambda*(-kp*(ee_x - ee_xdesired) - kv*(ee_v)) + p;

            N = robot->nullspaceMatrix(Jv);
            MatrixXd Kv = 25.0* MatrixXd::Identity(dof,dof);

            // Part i.
            control_torques = Jv.transpose()*F - N.transpose() * robot->M() * (Kv * (robot_dq));

            // Part ii.
            Lambda = MatrixXd::Identity(3,3);
            F = Lambda*(-kp*(ee_x - ee_xdesired) - kv*(ee_v)) + p;

            control_torques = Jv.transpose()*F - N.transpose() * robot->M() * (Kv * (robot_dq));

            // Part iii.
            Lambda = robot->taskInertiaMatrix(Jv);
            F = Lambda*(-kp*(ee_x - ee_xdesired) - kv*(ee_v)) + p;
            VectorXd q_desired (dof);
            q_desired << 0,0,0,0,0,0,0;
            MatrixXd KpJ = 400.0*MatrixXd::Identity(dof,dof);
            // KpJ(1,1) = 0.01;
            // KpJ(2,2) = 1.0;
            // KpJ(3,3) = 0.01;
            // KpJ(4,4) = 1.0;
            // KpJ(5,5) = 0.01;
            MatrixXd KvJ = 50.0*MatrixXd::Identity(dof,dof);
            control_torques = Jv.transpose()*F + N.transpose() * robot->M() * (-KpJ * (robot_q - q_desired) - KvJ * (robot_dq));

            // Part iv.
            control_torques = control_torques + N.transpose() * (robot->M() * (-KpJ * (robot_q - q_desired) - KvJ * (robot_dq)) + robot->jointGravityVector() );


            cout << ee_x.transpose() << endl;
            // cout << robot_q.transpose() << endl;

            // control_torques.setZero();
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
