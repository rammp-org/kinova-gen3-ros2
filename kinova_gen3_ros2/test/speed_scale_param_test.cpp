// The global_speed_scale parameter (driver #114): the startup value reaches
// the sink, a live set reaches it, a refused value fails the set with a
// reason, and the "Speed scale" diagnostics. No robot, no spinning: parameter
// sets on the node run the callback synchronously.
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "kinova_gen3_ros2/speed_scale_param.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"

using kinova_gen3_ros2::SpeedScaleParam;
namespace ki = kinova::interface;

namespace {
// Refuses as core's Supervisor::set_speed_override does (non-finite, or outside
// [kMinSpeedScale, 1.0]), plus whatever `refuse_all` says, with its own text,
// so a test can tell the sink's reason from rclcpp's.
struct FakeSpeedSink : public ki::CommandSink {
  std::vector<double> sets; // accepted, in order
  bool refuse_all = false;
  ki::SpeedResult
  on_set_speed_override(const ki::SpeedOverrideRequest &r) override {
    if (refuse_all)
      return {false, "fake: refused"};
    if (!std::isfinite(r.scale))
      return {false, "speed override must be finite"};
    if (r.scale < ki::kMinSpeedScale || r.scale > 1.0)
      return {false, "speed override out of range"};
    sets.push_back(r.scale);
    return {true, ""};
  }
  ki::GoalResponse on_trajectory_goal(const ki::TrajectoryGoal &) override {
    return ki::GoalResponse::kAccept;
  }
  void on_trajectory_accepted(const ki::GoalId &,
                              const ki::TrajectoryGoal &) override {}
  ki::CancelResponse on_trajectory_cancel(const ki::CancelRequest &) override {
    return ki::CancelResponse::kAccept;
  }
  ki::GainsResult on_set_gains(const ki::GainsRequest &) override {
    return {true, ""};
  }
  ki::ArmState on_query_state() override { return {}; }
  void on_halt(ki::HaltReason) override {}
};

rclcpp::Node::SharedPtr
make_node(const std::string &name,
          std::vector<rclcpp::Parameter> overrides = {}) {
  rclcpp::NodeOptions o;
  o.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, o);
}
} // namespace

class SpeedScaleParamTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(SpeedScaleParamTest, DefaultsToFullSpeedAndAppliesIt) {
  FakeSpeedSink sink;
  auto node = make_node("speed_default");
  SpeedScaleParam p(node, sink);
  ASSERT_EQ(sink.sets.size(), 1u);
  EXPECT_DOUBLE_EQ(sink.sets.back(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter(SpeedScaleParam::kName).as_double(),
                   1.0);
}

TEST_F(SpeedScaleParamTest, TheStartupValueIsApplied) {
  FakeSpeedSink sink;
  auto node = make_node("speed_startup",
                        {rclcpp::Parameter(SpeedScaleParam::kName, 0.4)});
  SpeedScaleParam p(node, sink);
  ASSERT_EQ(sink.sets.size(), 1u);
  EXPECT_DOUBLE_EQ(sink.sets.back(), 0.4);
  EXPECT_DOUBLE_EQ(p.value(), 0.4);
}

TEST_F(SpeedScaleParamTest, AStartupValueTheSinkRefusesIsFatal) {
  FakeSpeedSink sink;
  sink.refuse_all = true;
  auto node = make_node("speed_startup_refused");
  EXPECT_THROW(SpeedScaleParam(node, sink), std::invalid_argument);
}

TEST_F(SpeedScaleParamTest, AnOutOfRangeStartupValueIsFatal) {
  FakeSpeedSink sink;
  auto node = make_node("speed_startup_range",
                        {rclcpp::Parameter(SpeedScaleParam::kName, 1.5)});
  EXPECT_ANY_THROW(SpeedScaleParam(node, sink));
  EXPECT_TRUE(sink.sets.empty());
}

TEST_F(SpeedScaleParamTest, ALiveSetReachesTheSink) {
  FakeSpeedSink sink;
  auto node = make_node("speed_live");
  SpeedScaleParam p(node, sink);
  const auto r =
      node->set_parameter(rclcpp::Parameter(SpeedScaleParam::kName, 0.3));
  EXPECT_TRUE(r.successful) << r.reason;
  EXPECT_DOUBLE_EQ(sink.sets.back(), 0.3);
  EXPECT_DOUBLE_EQ(p.value(), 0.3);
  // And back up: anyone may raise it.
  EXPECT_TRUE(
      node->set_parameter(rclcpp::Parameter(SpeedScaleParam::kName, 1.0))
          .successful);
  EXPECT_DOUBLE_EQ(sink.sets.back(), 1.0);
}

TEST_F(SpeedScaleParamTest, AnOutOfRangeSetIsRejectedWithAReason) {
  FakeSpeedSink sink;
  auto node = make_node("speed_range");
  SpeedScaleParam p(node, sink);
  for (double bad : {1.5, 0.0, -0.2, 0.001}) {
    const auto r =
        node->set_parameter(rclcpp::Parameter(SpeedScaleParam::kName, bad));
    EXPECT_FALSE(r.successful) << bad;
    EXPECT_FALSE(r.reason.empty()) << bad;
  }
  EXPECT_EQ(sink.sets.size(), 1u) << "only the startup value reached the sink";
  EXPECT_DOUBLE_EQ(node->get_parameter(SpeedScaleParam::kName).as_double(),
                   1.0);
}

TEST_F(SpeedScaleParamTest, ARefusalCarriesTheDriversReason) {
  FakeSpeedSink sink;
  auto node = make_node("speed_reason");
  SpeedScaleParam p(node, sink);
  sink.refuse_all = true;
  const auto r =
      node->set_parameter(rclcpp::Parameter(SpeedScaleParam::kName, 0.5));
  EXPECT_FALSE(r.successful);
  EXPECT_EQ(r.reason, "fake: refused");
  EXPECT_DOUBLE_EQ(node->get_parameter(SpeedScaleParam::kName).as_double(), 1.0)
      << "a refused set leaves the parameter as it was";
  EXPECT_DOUBLE_EQ(p.value(), 1.0);
  // NaN passes rclcpp's range check; core refuses it.
  sink.refuse_all = false;
  const auto n = node->set_parameter(rclcpp::Parameter(
      SpeedScaleParam::kName, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(n.successful);
}

TEST_F(SpeedScaleParamTest, ADifferentParameterIsLeftAlone) {
  FakeSpeedSink sink;
  auto node = make_node("speed_other");
  SpeedScaleParam p(node, sink);
  node->declare_parameter("something_else", 1);
  EXPECT_TRUE(
      node->set_parameter(rclcpp::Parameter("something_else", 2)).successful);
  EXPECT_EQ(sink.sets.size(), 1u);
}

TEST(SpeedScaleDiagnostics, ReportsTheValueAtOk) {
  diagnostic_updater::DiagnosticStatusWrapper stat;
  kinova_gen3_ros2::speed_scale_diagnostics(0.5, stat);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_NE(stat.message.find("0.50"), std::string::npos) << stat.message;
  bool found = false;
  for (const auto &kv : stat.values)
    if (kv.key == SpeedScaleParam::kName) {
      found = true;
      EXPECT_EQ(kv.value, "0.50");
    }
  EXPECT_TRUE(found);
}
