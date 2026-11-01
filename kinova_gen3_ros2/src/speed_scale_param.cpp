#include "kinova_gen3_ros2/speed_scale_param.h"
#include <cstdio>
#include <stdexcept>
#include <string>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "kinova_lowlevel/interface/trajectory_executor.h" // kMinSpeedScale
namespace kinova_gen3_ros2 {

SpeedScaleParam::SpeedScaleParam(rclcpp::Node::SharedPtr node,
                                 kinova::interface::CommandSink &sink)
    : node_(std::move(node)), sink_(sink) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description =
      "Global speed scale: slows every arm motion (trajectories, velocity, "
      "twist and position streams) by this factor. Not torque streams or the "
      "gripper. Live; the params-file value applies at startup.";
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = kinova::interface::kMinSpeedScale;
  range.to_value = 1.0;
  d.floating_point_range.push_back(range);
  const double start = node_->declare_parameter(kName, 1.0, d);

  kinova::interface::SpeedOverrideRequest req;
  req.scale = start;
  const kinova::interface::SpeedResult r = sink_.on_set_speed_override(req);
  if (!r.accepted)
    throw std::invalid_argument(std::string(kName) + ": " + r.message);
  value_.store(start);
  // Registered AFTER the startup value is applied: declare_parameter does not
  // run it, and a refusal above must not leave a half-wired callback behind.
  cb_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &p) { return on_set(p); });

  // Its own Updater, as the other servers each own one.
  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Speed scale", this, &SpeedScaleParam::diagnostics);
  RCLCPP_INFO(node_->get_logger(), "%s = %.2f", kName, start);
}

rcl_interfaces::msg::SetParametersResult
SpeedScaleParam::on_set(const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  for (const auto &p : params) {
    if (p.get_name() != kName)
      continue;
    // Humble has no post-set callback, so the value is applied here; rclcpp
    // has already checked the type and the descriptor's range.
    kinova::interface::SpeedOverrideRequest req;
    req.scale = p.as_double();
    const kinova::interface::SpeedResult r = sink_.on_set_speed_override(req);
    if (!r.accepted) {
      res.successful = false;
      res.reason = r.message;
      return res;
    }
    value_.store(req.scale);
    RCLCPP_INFO(node_->get_logger(), "%s -> %.2f", kName, req.scale);
  }
  return res;
}

void SpeedScaleParam::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  speed_scale_diagnostics(value_.load(), stat);
}

void speed_scale_diagnostics(
    double scale, diagnostic_updater::DiagnosticStatusWrapper &stat) {
  char b[64];
  std::snprintf(b, sizeof b, "%.2f", scale);
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
               std::string("arm motions at ") + b + "x");
  stat.add(SpeedScaleParam::kName, b);
  stat.add("applies to",
           "trajectories, velocity/twist and position streams; not torque "
           "streams or the gripper");
}
} // namespace kinova_gen3_ros2
