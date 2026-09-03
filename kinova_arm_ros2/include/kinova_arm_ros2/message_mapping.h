#pragma once
#include "kinova_arm_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"
#include "kinova_lowlevel/interface/value_types.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
namespace kinova_arm_ros2 {
using ExecuteJointTrajectory = kinova_arm_interfaces::action::ExecuteJointTrajectory;
using GoToEEPose = kinova_arm_interfaces::action::GoToEEPose;
kinova::interface::TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g);
ExecuteJointTrajectory::Feedback to_feedback_msg(const kinova::interface::GoalId& id,
                                                 const kinova::interface::TrajectoryFeedback& fb);
ExecuteJointTrajectory::Result to_result_msg(const kinova::interface::TrajectoryResult& r);

kinova::interface::TrajectoryGoal to_trajectory_goal(
    const trajectory_msgs::msg::JointTrajectory& traj);
GoToEEPose::Feedback to_goto_feedback_msg(const kinova::interface::TrajectoryFeedback& fb);
GoToEEPose::Result   to_goto_result_msg(const kinova::interface::TrajectoryResult& r);

// robotiq_85_left_knuckle_joint's URDF upper limit. The gripper's ONE actuated DOF;
// robot_state_publisher derives the five mimics from it (verified 2026-09-03).
inline constexpr double kKnuckleUpperRad = 0.8;

// Core reports 0 (open) .. 1 (closed); sensor_msgs/JointState wants radians.
double gripper_to_knuckle_rad(float normalized);

kinova::interface::GripperSetpoint to_gripper_setpoint(
    const kinova_arm_interfaces::msg::GripperSetpoint& m);
kinova_arm_interfaces::msg::GripperState to_gripper_state_msg(
    const kinova::interface::GripperState& g);
}  // namespace kinova_arm_ros2
