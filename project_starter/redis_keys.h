// #pragma once

// // - read:
// const std::string JOINT_ANGLES_KEY = "sai::sensors::PANDA::joint_positions";
// const std::string JOINT_VELOCITIES_KEY = "sai::sensors::PANDA::joint_velocities";
// // - write
// const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::commands::PANDA::control_torques";
// const std::string GRAVITY_COMP_ENABLED_KEY = "sai::simviz::gravity_comp_enabled";

/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

std::string JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
 const std::string CONTROLLER_RUNNING_KEY = "sai::sim::PANDA::controller";
 
 const std::string GRIPPER_JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
//  const std::string GRIPPER_JOINT_VELOCITIES_KEY = "sai::sim::panda_gripper::sensors::dq";

 const std::string EE_FORCES_KEY = "sai::sensors::PANDA::ft_sensor::end-effector::force";
 const std::string EE_MOMENTS_KEY = "sai::sensors::PANDA::ft_sensor::end-effector::moment";

 const std::string EE_POSITION_KEY = "sai::sim::PANDA::end-effector::position";
 const std::string EE_VELOCITY_KEY = "sai::sim::PANDA::end-effector::velocity";

 std::string BALL_POSITION_KEY = "sai::sim::BALL::sensors::position";
 std::string BALL_VELOCITY_KEY = "sai::sim::BALL::sensors::velocity";
 std::string BALL_APEX_KEY = "sai::sim::BALL::sensors::apex";

 std::string MASS_MATRIX_KEY;
 