#pragma once
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_gen3_ros2 {

// A JointVec as the JointTrajectoryPoint the action Feedback/Result fields use.
// Shared so the templated PlannedMoveServer and message_mapping agree on one
// definition.
inline trajectory_msgs::msg::JointTrajectoryPoint
vec_to_point(const kinova::JointVec &v) {
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.resize(kinova::kNumJoints);
  for (int i = 0; i < kinova::kNumJoints; ++i)
    p.positions[i] = v[i];
  return p;
}

} // namespace kinova_gen3_ros2
