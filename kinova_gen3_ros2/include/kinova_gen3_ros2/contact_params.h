// kinova_gen3_ros2/include/kinova_gen3_ros2/contact_params.h
#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/float_mode.h"
#include "kinova_lowlevel/interface/contact.h"
namespace kinova_gen3_ros2 {

// The contact detector's startup configuration, read from ROS parameters.
struct ContactNodeConfig {
  std::string detection; // "enforce" | "observe" | "off"
  kinova::ContactMonitorConfig monitor;
  kinova::interface::ContactProfiles profiles;
  kinova::FloatParams floating;
  bool debug_topic = false; // publish /contact_monitor/residual
};

// Declares the contact parameters, all READ-ONLY: a ROS parameter set carries
// no control token, so a live edit would let any process retune the detector.
// Live tuning is PROFILE_CUSTOM on /set_contact_thresholds, which is gated.
//
//   contact_detection                    enforce | observe | off (enforce)
//   contact_gain                         observer K, rad/s (25)
//   contact_debounce_cycles              cycles over θ to trip (5)
//   contact_thresholds_sensitive         double[7], N·m
//   contact_thresholds_default           double[7], N·m
//   contact_thresholds_loose             double[7], N·m
//   contact_thresholds_ceiling           double[7], N·m, bounds everything
//   float_damping                        double[7], N·m·s/rad
//   float_spring                         double[7], N·m/rad
//   float_drag_band_deg                  anchor drag band, deg (2)
//   contact_debug_topic                  /contact_monitor/residual (false)
//
// Throws std::invalid_argument naming the parameter (and the joint, for a
// threshold) on anything invalid. Nothing is clamped.
ContactNodeConfig declare_contact_params(rclcpp::Node &node);
} // namespace kinova_gen3_ros2
