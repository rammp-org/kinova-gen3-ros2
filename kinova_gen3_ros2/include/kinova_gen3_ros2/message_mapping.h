#pragma once
#include "rammp_arm_interfaces/action/execute_joint_trajectory.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_pose.hpp"
#include "rammp_arm_interfaces/msg/gains_spec.hpp"
#include "rammp_arm_interfaces/msg/gripper_setpoint.hpp"
#include "rammp_arm_interfaces/msg/gripper_state.hpp"
#include "kinova_lowlevel/interface/value_types.h"
#include <cstdint>
#include <optional>
#include <string>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
namespace kinova_gen3_ros2 {
using ExecuteJointTrajectory =
    rammp_arm_interfaces::action::ExecuteJointTrajectory;
using GoToEEPose = rammp_arm_interfaces::action::GoToEEPose;
kinova::interface::TrajectoryGoal
to_trajectory_goal(const ExecuteJointTrajectory::Goal &g);

// 1 => kImpedance, anything else => kPosition. Total on purpose -- callers
// refuse an unknown byte via mode_gains_rejection BEFORE mapping, so this
// never has to invent a meaning for one.
kinova::interface::ControlModeKind to_control_mode(uint8_t m);

// GainsSpec message -> core's GainsSpec. The default message (profile
// PROFILE_SESSION_DEFAULT) maps to kSessionDefault with NO custom read; the
// custom fields are copied iff profile == PROFILE_CUSTOM. (The old shape set
// has_gains with the message's zero-filled numbers on every impedance goal,
// so "I didn't say" arrived as "zero stiffness".)
kinova::interface::GainsSpec
to_gains_spec(const rammp_arm_interfaces::msg::GainsSpec &m);

// Why this (control_mode, gains) pair is unacceptable, or nullopt. Mirrors
// the driver's accept-time checks -- unknown bytes, gains on a non-impedance
// command, custom gains out of bounds -- so the reason reaches the node log /
// service response (the driver's GoalResponse carries no message). For a
// streaming open, pass 1 for an impedance controller and 0 otherwise.
std::optional<std::string>
mode_gains_rejection(uint8_t control_mode,
                     const rammp_arm_interfaces::msg::GainsSpec &g);
ExecuteJointTrajectory::Feedback
to_feedback_msg(const kinova::interface::GoalId &id,
                const kinova::interface::TrajectoryFeedback &fb);
ExecuteJointTrajectory::Result
to_result_msg(const kinova::interface::TrajectoryResult &r);

// The planner-output overload (GoTo* servers). control_mode/gains default to
// today's behaviour: a stiff position execution under the session default.
kinova::interface::TrajectoryGoal
to_trajectory_goal(const trajectory_msgs::msg::JointTrajectory &traj,
                   double speed_scale = 1.0,
                   kinova::interface::ControlModeKind control_mode =
                       kinova::interface::ControlModeKind::kPosition,
                   const kinova::interface::GainsSpec &gains = {});

// Why this speed_scale is unacceptable, or nullopt if it is fine. The driver
// refuses the same range but its GoalResponse carries no message, so the
// reason is produced here. Callers log it server-side: an action rejection has
// no payload, so the client sees a bare rejection (see kinova-gen3-ros2#39).
std::optional<std::string> speed_scale_rejection(double s);
GoToEEPose::Feedback
to_goto_feedback_msg(const kinova::interface::TrajectoryFeedback &fb);
GoToEEPose::Result
to_goto_result_msg(const kinova::interface::TrajectoryResult &r);

// robotiq_85_left_knuckle_joint's URDF upper limit. The gripper's ONE actuated
// DOF; robot_state_publisher derives the five mimics from it (verified
// 2026-09-03).
inline constexpr double kKnuckleUpperRad = 0.8;

// Core reports 0 (open) .. 1 (closed); sensor_msgs/JointState wants radians.
double gripper_to_knuckle_rad(float normalized);

kinova::interface::GripperSetpoint
to_gripper_setpoint(const rammp_arm_interfaces::msg::GripperSetpoint &m);
rammp_arm_interfaces::msg::GripperState
to_gripper_state_msg(const kinova::interface::GripperState &g);
} // namespace kinova_gen3_ros2
