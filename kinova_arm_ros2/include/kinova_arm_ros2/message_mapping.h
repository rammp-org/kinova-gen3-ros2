#pragma once
#include "kinova_arm_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova_arm_ros2 {
using ExecuteJointTrajectory = kinova_arm_interfaces::action::ExecuteJointTrajectory;
kinova::interface::TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g);
ExecuteJointTrajectory::Feedback to_feedback_msg(const kinova::interface::GoalId& id,
                                                 const kinova::interface::TrajectoryFeedback& fb);
ExecuteJointTrajectory::Result to_result_msg(const kinova::interface::TrajectoryResult& r);
}  // namespace kinova_arm_ros2
