// The contact detector's startup parameters: defaults and fail-loud refusals.
#include <stdexcept>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_ros2/contact_params.h"

using kinova::JointVec;
using kinova_gen3_ros2::declare_contact_params;

namespace {
JointVec vec(std::initializer_list<double> v) {
  JointVec out;
  int i = 0;
  for (double x : v)
    out[i++] = x;
  return out;
}
rclcpp::Node::SharedPtr
node_with(const std::string &name,
          std::vector<rclcpp::Parameter> overrides = {}) {
  rclcpp::NodeOptions o;
  o.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, o);
}
// Message of the invalid_argument declare_contact_params throws, or "".
std::string refusal(rclcpp::Node &n) {
  try {
    declare_contact_params(n);
  } catch (const std::invalid_argument &e) {
    return e.what();
  }
  return "";
}
} // namespace

class ContactParams : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

TEST_F(ContactParams, DefaultsMatchSpec) {
  auto n = node_with("contact_params_defaults");
  const auto c = declare_contact_params(*n);
  EXPECT_EQ(c.detection, "enforce");
  EXPECT_EQ(c.monitor.detection, kinova::ContactDetection::kEnforce);
  EXPECT_DOUBLE_EQ(c.monitor.gain, 25.0);
  EXPECT_EQ(c.monitor.debounce_cycles, 5);
  EXPECT_EQ(c.profiles.sensitive, vec({8, 8, 6, 6, 2, 2, 2}));
  EXPECT_EQ(c.profiles.normal, vec({16, 16, 12, 12, 4, 4, 4}));
  EXPECT_EQ(c.profiles.loose, vec({24, 24, 18, 18, 6, 6, 6}));
  EXPECT_EQ(c.profiles.ceiling, vec({30, 30, 22, 22, 7, 7, 7}));
  // The monitor starts on the default profile under the same ceiling, which
  // the Supervisor requires.
  EXPECT_EQ(c.monitor.initial_theta, c.profiles.normal);
  EXPECT_EQ(c.monitor.ceiling, c.profiles.ceiling);
  EXPECT_EQ(c.floating.damping, vec({5, 5, 5, 5, 1.5, 1.5, 1.5}));
  EXPECT_EQ(c.floating.spring, vec({28.6, 28.6, 28.6, 28.6, 8.6, 8.6, 8.6}));
  EXPECT_DOUBLE_EQ(c.floating.drag_band_rad, 2.0 * kinova::kDeg2Rad);
  EXPECT_EQ(c.floating.torque_limit, vec({39, 39, 39, 39, 9, 9, 9}));
  EXPECT_FALSE(c.debug_topic);

  // Read-only: a live set is refused, not silently ignored.
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("contact_detection", "off"))
                   .successful);
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("contact_debug_topic", true))
                   .successful);
}

TEST_F(ContactParams, DebugTopicCanBeEnabled) {
  auto n = node_with("contact_params_debug",
                     {rclcpp::Parameter("contact_debug_topic", true)});
  EXPECT_TRUE(declare_contact_params(*n).debug_topic);
}

TEST_F(ContactParams, ObserveAndOffAreAccepted) {
  auto o = node_with("contact_params_observe",
                     {rclcpp::Parameter("contact_detection", "observe")});
  const auto c = declare_contact_params(*o);
  EXPECT_EQ(c.detection, "observe");
  EXPECT_EQ(c.monitor.detection, kinova::ContactDetection::kObserve);
  auto f = node_with("contact_params_off",
                     {rclcpp::Parameter("contact_detection", "off")});
  EXPECT_EQ(declare_contact_params(*f).detection, "off");
}

TEST_F(ContactParams, InvalidThrowsNamingParameter) {
  auto above = node_with(
      "contact_params_above",
      {rclcpp::Parameter("contact_thresholds_default",
                         std::vector<double>{16, 16, 12, 12, 4, 4, 7.5})});
  const std::string m = refusal(*above);
  EXPECT_NE(m.find("contact_thresholds_default"), std::string::npos) << m;
  EXPECT_NE(m.find("joint 7"), std::string::npos) << m;
  EXPECT_NE(m.find("ceiling 7"), std::string::npos) << m;

  auto short_vec =
      node_with("contact_params_short",
                {rclcpp::Parameter("contact_thresholds_loose",
                                   std::vector<double>{24, 24, 18})});
  const std::string s = refusal(*short_vec);
  EXPECT_NE(s.find("contact_thresholds_loose"), std::string::npos) << s;
  EXPECT_NE(s.find("7"), std::string::npos) << s;

  auto mode = node_with("contact_params_mode",
                        {rclcpp::Parameter("contact_detection", "on")});
  EXPECT_NE(refusal(*mode).find("contact_detection"), std::string::npos);

  auto gain = node_with("contact_params_gain",
                        {rclcpp::Parameter("contact_gain", 0.0)});
  EXPECT_NE(refusal(*gain).find("contact_gain"), std::string::npos);

  auto debounce = node_with("contact_params_debounce",
                            {rclcpp::Parameter("contact_debounce_cycles", 0)});
  EXPECT_NE(refusal(*debounce).find("contact_debounce_cycles"),
            std::string::npos);

  auto ceiling = node_with(
      "contact_params_ceiling",
      {rclcpp::Parameter("contact_thresholds_ceiling",
                         std::vector<double>{30, 30, 22, 22, 7, 7, -1})});
  EXPECT_NE(refusal(*ceiling).find("contact_thresholds_ceiling"),
            std::string::npos);

  auto damping = node_with(
      "contact_params_damping",
      {rclcpp::Parameter("float_damping",
                         std::vector<double>{5, 5, 5, -5, 1.5, 1.5, 1.5})});
  const std::string d = refusal(*damping);
  EXPECT_NE(d.find("float_damping"), std::string::npos) << d;
  EXPECT_NE(d.find("joint 4"), std::string::npos) << d;

  auto band = node_with("contact_params_band",
                        {rclcpp::Parameter("float_drag_band_deg", 0.0)});
  EXPECT_NE(refusal(*band).find("float_drag_band_deg"), std::string::npos);
}
