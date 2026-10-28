// kinova_gen3_ros2/include/kinova_gen3_ros2/link_server.h
#pragma once
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_lowlevel/arm_link.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's ArmLink: /clear_faults and the "Arm link" entry on
// /diagnostics. The link is how the node survives faults, disconnects and an
// arm that is off; this is how an operator sees which of those is happening and
// takes the arm back after a fault.
class LinkServer {
public:
  using Trigger = std_srvs::srv::Trigger;

  // arm: shown in the "waiting for the arm" message (the IP, or "sim").
  LinkServer(rclcpp::Node::SharedPtr node, kinova::ArmLink &link,
             std::string arm);

private:
  void on_clear(const std::shared_ptr<Trigger::Request>,
                std::shared_ptr<Trigger::Response>);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  rclcpp::Node::SharedPtr node_;
  kinova::ArmLink &link_;
  std::string arm_;
  rclcpp::CallbackGroup::SharedPtr clear_group_;
  rclcpp::Service<Trigger>::SharedPtr clear_srv_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

// The "Arm link" diagnostics for one status, separate so it unit-tests without
// a running updater. kRunning is OK, kFaulted ERROR with the reason, kConnecting
// STALE ("waiting for the arm at <arm>").
void link_diagnostics(const kinova::LinkStatus &s, const std::string &arm,
                      diagnostic_updater::DiagnosticStatusWrapper &stat);

// How long /clear_faults waits for the clear to resolve before answering: a
// clear is ClearFaults + SetServoingMode + seed + 7 SetControlMode RPCs, each
// with KORTEX's 3 s default bound.
inline constexpr std::chrono::seconds kClearWait{10};
} // namespace kinova_gen3_ros2
