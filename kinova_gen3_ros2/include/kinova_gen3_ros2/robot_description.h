// kinova_gen3_ros2/include/kinova_gen3_ros2/robot_description.h
#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
namespace kinova_gen3_ros2 {

// arm_state_publisher (kinova_gen3_description's description.launch.py) renders
// the model once, at launch, from the robot's mount file and publishes it here,
// latched. The driver reads it rather than a URDF file, so it cannot disagree
// with TF or with cuRobo -- including about the mount rotation, which is what
// gravity compensation depends on.
inline constexpr char kRobotDescriptionTopic[] = "/arm/robot_description";
// The 2F-85's one actuated joint; robot_state_publisher derives its five
// mimics.
inline constexpr char kGripperJoint[] = "arm_robotiq_85_left_knuckle_joint";

// Block until one message arrives on kRobotDescriptionTopic (RELIABLE +
// TRANSIENT_LOCAL, so a description published before we subscribed still
// arrives), or until `timeout`. nullopt on timeout or shutdown. Spins `node` on
// a private executor: call it before the node joins any other executor.
std::optional<std::string>
wait_for_robot_description(const rclcpp::Node::SharedPtr &node,
                           std::chrono::milliseconds timeout);

// The names /joint_states carries: `arm_joints` (the model's arm joints in
// chain order -- Dynamics::joint_names()) followed by kGripperJoint. Throws
// std::runtime_error naming the joint if the URDF does not declare it:
// robot_state_publisher matches by NAME, and a name it does not know freezes
// that part of TF silently.
std::vector<std::string> joint_state_names(const std::string &urdf_xml,
                                           std::vector<std::string> arm_joints);
} // namespace kinova_gen3_ros2
