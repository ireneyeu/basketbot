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

 const std::string JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
 const std::string JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
 const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
 const std::string CONTROLLER_RUNNING_KEY = "sai::sim::PANDA::controller";
 
 const std::string GRIPPER_JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
//  const std::string GRIPPER_JOINT_VELOCITIES_KEY = "sai::sim::panda_gripper::sensors::dq";

 const std::string EE_FORCES = "sai::sensors::PANDA::ft_sensor::end-effector::force";
 const std::string EE_MOMENTS = "sai::sensors::PANDA::ft_sensor::end-effector::moment";