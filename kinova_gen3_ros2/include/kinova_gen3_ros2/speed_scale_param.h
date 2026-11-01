// kinova_gen3_ros2/include/kinova_gen3_ros2/speed_scale_param.h
#pragma once
#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's global speed scale (driver #114): the
// global_speed_scale parameter and the "Speed scale" entry on /diagnostics.
// One number in [0.01, 1.0] that slows every arm motion: trajectories and
// planned moves, joint velocity and twist streams, joint position streams.
// Torque streams and the gripper are unaffected.
//
// The parameter IS the value in force: a UI reads it, and watches
// /parameter_events for changes. Anyone may set it -- a parameter set carries
// no token, and core does not gate the scale.
//
// Holds a CommandSink& and nothing else from core, so it unit-tests against a
// fake.
class SpeedScaleParam {
public:
  static constexpr const char *kName = "global_speed_scale";

  // Declares global_speed_scale (double, default 1.0, range [0.01, 1.0] in the
  // descriptor) and applies its startup value -- the deployment's default,
  // from the params file -- through sink.on_set_speed_override. Throws
  // std::invalid_argument with core's reason if core refuses it, and rclcpp
  // throws if the value is outside the descriptor's range: fail loud at
  // startup. Afterwards every set of the parameter goes to the sink; a value
  // the sink refuses fails the set, and the result's reason is core's.
  SpeedScaleParam(rclcpp::Node::SharedPtr node,
                  kinova::interface::CommandSink &sink);

  double value() const { return value_.load(); }

private:
  rcl_interfaces::msg::SetParametersResult
  on_set(const std::vector<rclcpp::Parameter> &params);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::CommandSink &sink_;
  std::atomic<double> value_{
      1.0}; // last accepted; read by the diagnostics timer
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

// The "Speed scale" diagnostics for one value, separate so it unit-tests
// without a running updater. Always OK: the scale is a setting, not a fault.
void speed_scale_diagnostics(double scale,
                             diagnostic_updater::DiagnosticStatusWrapper &stat);
} // namespace kinova_gen3_ros2
