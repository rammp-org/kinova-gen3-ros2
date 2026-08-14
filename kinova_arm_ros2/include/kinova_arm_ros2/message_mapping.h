#pragma once
#include "kinova_arm_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
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
}  // namespace kinova_arm_ros2
