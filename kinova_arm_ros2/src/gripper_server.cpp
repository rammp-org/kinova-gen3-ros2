#include "kinova_arm_ros2/gripper_server.h"
#include "kinova_arm_ros2/message_mapping.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_arm_ros2 {

GripperServer::GripperServer(rclcpp::Node::SharedPtr node,
                             kinova::interface::GripperSink& sink,
                             bool expect_gripper)
    : node_(std::move(node)), sink_(sink), expect_gripper_(expect_gripper) {
  // Best-effort, KeepLast(1): identical to the six arm setpoint topics, because core's
  // semantics are identical -- setpoints are absolute and latest-wins, so dropping an
  // intermediate one is correct rather than a loss.
  const auto sp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  sp_sub_ = node_->create_subscription<GripperSetpointMsg>(
      "/setpoint/gripper", sp_qos,
      std::bind(&GripperServer::on_setpoint, this, std::placeholders::_1));
  state_pub_ = node_->create_publisher<GripperStateMsg>("gripper_state",
                                                        rclcpp::SensorDataQoS());
  // Its own Updater, exactly as ArbitrationServer and Ros2Backend each own one.
  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Gripper", this, &GripperServer::diagnostics);
}

void GripperServer::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  const auto g = sink_.on_query_gripper();
  if (expect_gripper_ && !g.present) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "no gripper reported, but expect_gripper is true");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 g.present ? "gripper present" : "no gripper (expected)");
  }
  stat.add("present", g.present);
  stat.add("position", g.position);
  stat.add("effort_fraction", g.effort);
  stat.add("current_a", g.current);
}

void GripperServer::on_setpoint(const GripperSetpointMsg::SharedPtr m) {
  // Straight through to the sink, which is the ARBITER: that is what makes the token
  // load-bearing. A bad token is counted by the Arbiter, not refused here.
  sink_.on_gripper_setpoint(to_gripper_setpoint(*m));
}

void GripperServer::publish_state(const builtin_interfaces::msg::Time& stamp) {
  auto msg = to_gripper_state_msg(sink_.on_query_gripper());
  msg.header.stamp = stamp;
  state_pub_->publish(msg);
}
}  // namespace kinova_arm_ros2
