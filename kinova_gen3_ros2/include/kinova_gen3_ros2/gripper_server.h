// kinova_gen3_ros2/include/kinova_gen3_ros2/gripper_server.h
#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "kinova_gen3_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/gripper_state.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's GripperSink: /setpoint/gripper in, /gripper_state out.
//
// Holds a GripperSink& and NOTHING else from core, so it unit-tests against a
// fake with no robot -- the same reason StreamServer holds only a StreamSink&.
//
// There is NO session here, unlike StreamServer. Arbiter::on_gripper_setpoint
// gates on admit(token) alone, so holding the arm's token is the whole
// prerequisite: the gripper rides the ARM's token by core's spec decision, one
// physical machine one holder.
class GripperServer {
public:
  using GripperSetpointMsg = kinova_gen3_interfaces::msg::GripperSetpoint;
  using GripperStateMsg = kinova_gen3_interfaces::msg::GripperState;

  // expect_gripper: "expected" cannot be inferred from the node's own model --
  // it loads the FROZEN 7-DOF URDF, where the Robotiq joints are type="fixed",
  // so its model never has a gripper regardless of the hardware. A robot
  // genuinely built without one sets this false and the diagnostics task
  // reports OK instead of WARN.
  GripperServer(rclcpp::Node::SharedPtr node,
                kinova::interface::GripperSink &sink, bool expect_gripper);

  // Pulls on_query_gripper() and publishes it. The caller supplies the stamp;
  // GripperState::stamp_s is QUERY time, not sample time, and is deliberately
  // discarded.
  void publish_state(const builtin_interfaces::msg::Time &stamp);

private:
  void on_setpoint(const GripperSetpointMsg::SharedPtr m);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::GripperSink &sink_;
  bool expect_gripper_;
  rclcpp::CallbackGroup::SharedPtr setpoint_group_;
  rclcpp::Subscription<GripperSetpointMsg>::SharedPtr sp_sub_;
  rclcpp::Publisher<GripperStateMsg>::SharedPtr state_pub_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};
} // namespace kinova_gen3_ros2
