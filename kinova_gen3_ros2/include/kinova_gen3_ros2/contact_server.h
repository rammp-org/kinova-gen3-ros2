// kinova_gen3_ros2/include/kinova_gen3_ros2/contact_server.h
#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rammp_arm_interfaces/srv/set_contact_thresholds.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's contact detector: /set_contact_thresholds, the
// "Contact detection" entry on /diagnostics and, with contact_debug_topic, the
// residual on /contact_monitor/residual.
//
// Holds a CommandSink& (the ARBITER, so the threshold set is token-gated) and
// nothing else from core, so it unit-tests against a fake.
class ContactServer {
public:
  using SetContactThresholds = rammp_arm_interfaces::srv::SetContactThresholds;

  // detection: the contact_detection parameter ("enforce" | "observe" |
  // "off"), shown in diagnostics; with "off" no monitor exists and the sink
  // reports enabled = false.
  // debug_topic: the contact_debug_topic parameter. When true and detection
  // is not "off", publishes sensor_msgs/JointState on
  // /contact_monitor/residual at 50 Hz: `name` = arm_joints (the seven arm
  // joint names /joint_states uses), `effort` = r in N·m. Otherwise the topic
  // is not created.
  ContactServer(rclcpp::Node::SharedPtr node,
                kinova::interface::CommandSink &gated, std::string detection,
                bool debug_topic = false,
                std::vector<std::string> arm_joints = {});

private:
  void publish_residual();
  void on_set(const std::shared_ptr<SetContactThresholds::Request>,
              std::shared_ptr<SetContactThresholds::Response>);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::CommandSink &sink_;
  std::string detection_;
  rclcpp::CallbackGroup::SharedPtr set_group_;
  rclcpp::Service<SetContactThresholds>::SharedPtr set_srv_;
  std::vector<std::string> arm_joints_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr residual_pub_;
  rclcpp::TimerBase::SharedPtr residual_timer_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

// Maps the wire request onto core's. Returns the reason to refuse, or nullopt:
// an unknown profile byte, `custom` not exactly 7 values for PROFILE_CUSTOM (or
// not empty for a named profile), or a non-finite custom value. Everything else
// (<= 0, above the ceiling, the token) is core's to refuse.
std::optional<std::string>
to_contact_request(const ContactServer::SetContactThresholds::Request &req,
                   kinova::interface::ContactThresholdsRequest &out);

// "sensitive" | "default" | "loose" | "custom".
std::string to_string(kinova::interface::ContactProfile p);

// The "Contact detection" diagnostics for one state, separate so it unit-tests
// without a running updater.
void contact_diagnostics(const kinova::interface::ContactState &s,
                         const std::string &detection,
                         diagnostic_updater::DiagnosticStatusWrapper &stat);
} // namespace kinova_gen3_ros2
