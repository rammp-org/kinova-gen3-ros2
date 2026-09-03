#include "kinova_gen3_ros2/gripper_server.h"
#include "kinova_gen3_ros2/message_mapping.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_gen3_ros2 {

GripperServer::GripperServer(rclcpp::Node::SharedPtr node,
                             kinova::interface::GripperSink& sink,
                             bool expect_gripper)
    : node_(std::move(node)), sink_(sink), expect_gripper_(expect_gripper) {
  // Best-effort, KeepLast(1): identical to the six arm setpoint topics, because core's
  // semantics are identical -- setpoints are absolute and latest-wins, so dropping an
  // intermediate one is correct rather than a loss.
  const auto sp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  // Its OWN mutually-exclusive group, isolated from the node's DEFAULT group -- which
  // also carries /acquire_control, /release_control, /revoke_control and the
  // execute_joint_trajectory goal/cancel callbacks. on_setpoint blocks on the Arbiter
  // mutex, and StreamServer::on_open holds that mutex for a 250 ms mode-settle sleep;
  // sharing the default group would stall an operator's /revoke_control or a
  // trajectory cancel behind that sleep. Same reasoning as StreamServer's
  // setpoint_group_ and ArbitrationServer's estop_group_.
  setpoint_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sp_opts;
  sp_opts.callback_group = setpoint_group_;
  sp_sub_ = node_->create_subscription<GripperSetpointMsg>(
      "/setpoint/gripper", sp_qos,
      std::bind(&GripperServer::on_setpoint, this, std::placeholders::_1), sp_opts);
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
}  // namespace kinova_gen3_ros2
