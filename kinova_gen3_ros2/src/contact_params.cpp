#include "kinova_gen3_ros2/contact_params.h"
#include <cmath>
#include <stdexcept>
#include <vector>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
namespace kinova_gen3_ros2 {
namespace {
using kinova::JointVec;

rcl_interfaces::msg::ParameterDescriptor read_only(const std::string &what) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;
  d.description = what + " Launch-time only.";
  return d;
}

JointVec declare_vec(rclcpp::Node &node, const std::string &name,
                     const JointVec &def, const std::string &what) {
  const std::vector<double> v = node.declare_parameter(
      name, std::vector<double>(def.data(), def.data() + def.size()),
      read_only(what));
  if (v.size() != static_cast<size_t>(kinova::kNumJoints))
    throw std::invalid_argument(
        name + " must have exactly " + std::to_string(kinova::kNumJoints) +
        " values (joints 1..7), got " + std::to_string(v.size()));
  JointVec out;
  for (int i = 0; i < kinova::kNumJoints; ++i)
    out[i] = v[i];
  return out;
}

// Every entry finite and >= 0, or a message naming the joint.
void require_non_negative(const std::string &name, const JointVec &v) {
  for (int i = 0; i < kinova::kNumJoints; ++i)
    if (!(std::isfinite(v[i]) && v[i] >= 0.0))
      throw std::invalid_argument(name + ": joint " + std::to_string(i + 1) +
                                  " value " + std::to_string(v[i]) +
                                  " must be finite and >= 0");
}
} // namespace

ContactNodeConfig declare_contact_params(rclcpp::Node &node) {
  ContactNodeConfig c;
  c.detection = node.declare_parameter(
      "contact_detection", std::string("enforce"),
      read_only("enforce | observe | off. observe reports would-be trips and "
                "never stops the arm; off attaches no monitor."));
  if (c.detection != "enforce" && c.detection != "observe" &&
      c.detection != "off")
    throw std::invalid_argument(
        "contact_detection must be 'enforce', 'observe' or 'off', got '" +
        c.detection + "'");
  c.monitor.detection = c.detection == "observe"
                            ? kinova::ContactDetection::kObserve
                            : kinova::ContactDetection::kEnforce;

  c.monitor.gain = node.declare_parameter(
      "contact_gain", 25.0, read_only("Momentum-observer gain K, rad/s."));
  if (!(std::isfinite(c.monitor.gain) && c.monitor.gain > 0.0))
    throw std::invalid_argument("contact_gain must be finite and > 0, got " +
                                std::to_string(c.monitor.gain));
  const int64_t debounce = node.declare_parameter(
      "contact_debounce_cycles", int64_t{5},
      read_only("Consecutive RT cycles over threshold to trip."));
  if (debounce < 1)
    throw std::invalid_argument("contact_debounce_cycles must be >= 1, got " +
                                std::to_string(debounce));
  c.monitor.debounce_cycles = static_cast<int>(debounce);

  const auto vec = [](std::initializer_list<double> v) {
    JointVec out;
    int i = 0;
    for (double x : v)
      out[i++] = x;
    return out;
  };
  const std::string theta_what = "Contact thresholds, N·m, joints 1..7.";
  c.profiles.sensitive =
      declare_vec(node, "contact_thresholds_sensitive",
                  vec({4, 4, 3, 3, 1.2, 1.2, 1.2}), theta_what);
  c.profiles.normal = declare_vec(node, "contact_thresholds_default",
                                  vec({8, 8, 6, 6, 2, 2, 2}), theta_what);
  c.profiles.loose = declare_vec(node, "contact_thresholds_loose",
                                 vec({20, 20, 15, 15, 5, 5, 5}), theta_what);
  c.profiles.ceiling = declare_vec(
      node, "contact_thresholds_ceiling", vec({30, 30, 22, 22, 7, 7, 7}),
      "Per-joint ceiling bounding every profile and custom value, N·m.");
  // The ceiling on its own first, so a bad one is not reported as a bad
  // profile; then each profile against it. Core's message names the joint and
  // the limit.
  const auto ceil = kinova::validate_contact_thresholds(c.profiles.ceiling,
                                                        c.profiles.ceiling);
  if (!ceil.ok)
    throw std::invalid_argument("contact_thresholds_ceiling: " + ceil.message);
  const struct {
    const char *name;
    const JointVec &theta;
  } named[] = {{"contact_thresholds_sensitive", c.profiles.sensitive},
               {"contact_thresholds_default", c.profiles.normal},
               {"contact_thresholds_loose", c.profiles.loose}};
  for (const auto &n : named) {
    const auto check =
        kinova::validate_contact_thresholds(n.theta, c.profiles.ceiling);
    if (!check.ok)
      throw std::invalid_argument(std::string(n.name) + ": " + check.message);
  }
  c.monitor.initial_theta = c.profiles.normal;
  c.monitor.ceiling = c.profiles.ceiling;

  c.floating.damping = declare_vec(node, "float_damping", c.floating.damping,
                                   "Float-mode joint damping, N·m·s/rad.");
  require_non_negative("float_damping", c.floating.damping);
  c.floating.spring = declare_vec(node, "float_spring", c.floating.spring,
                                  "Float-mode anchor spring, N·m/rad.");
  require_non_negative("float_spring", c.floating.spring);
  const double band_deg = node.declare_parameter(
      "float_drag_band_deg", 2.0,
      read_only("How far the float anchor trails q, deg."));
  if (!(std::isfinite(band_deg) && band_deg > 0.0))
    throw std::invalid_argument(
        "float_drag_band_deg must be finite and > 0, got " +
        std::to_string(band_deg));
  c.floating.drag_band_rad = band_deg * kinova::kDeg2Rad;
  return c;
}
} // namespace kinova_gen3_ros2
