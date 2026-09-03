#include <cmath>
#include <functional>
#include <stdexcept>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_ros2/gripper_server.h"
#include "kinova_gen3_ros2/ros2_backend.h"
#include "kinova_gen3_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/gripper_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fake_gripper_sink.h"

using namespace std::chrono_literals;

class GripperServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("gripper_server_test");
  }
  void spin_for(std::chrono::milliseconds d) {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(2ms);
    }
  }
  rclcpp::Node::SharedPtr node_;
};

TEST_F(GripperServerTest, ASetpointReachesTheSinkWithAllThreeFields) {
  FakeGripperSink sink;
  kinova_gen3_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  auto pub = node_->create_publisher<kinova_gen3_interfaces::msg::GripperSetpoint>(
      "/setpoint/gripper", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  spin_for(200ms);   // let discovery settle

  kinova_gen3_interfaces::msg::GripperSetpoint m;
  m.position = 0.6f; m.speed = 0.4f; m.force = 0.2f; m.token[0] = 3;
  for (int i = 0; i < 20 && sink.count() == 0; ++i) { pub->publish(m); spin_for(20ms); }

  ASSERT_GE(sink.count(), 1u);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.position, 0.6f);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.speed, 0.4f);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.force, 0.2f);
  EXPECT_EQ(sink.setpoints.back().token[0], 3);
}

TEST_F(GripperServerTest, PublishStateReportsWhatTheSinkSays) {
  FakeGripperSink sink;
  sink.state.position = 0.3f; sink.state.effort = 0.05f;
  sink.state.current = 0.05f; sink.state.present = true;
  kinova_gen3_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  kinova_gen3_interfaces::msg::GripperState got;
  bool seen = false;
  auto sub = node_->create_subscription<kinova_gen3_interfaces::msg::GripperState>(
      "/gripper_state", rclcpp::SensorDataQoS(),
      [&](kinova_gen3_interfaces::msg::GripperState::SharedPtr msg) { got = *msg; seen = true; });
  spin_for(200ms);

  for (int i = 0; i < 20 && !seen; ++i) {
    server.publish_state(node_->now());
    spin_for(20ms);
  }
  ASSERT_TRUE(seen);
  EXPECT_FLOAT_EQ(got.position, 0.3f);
  EXPECT_FLOAT_EQ(got.effort, 0.05f);
  EXPECT_TRUE(got.present);
}

namespace {
// diagnostic_updater::Updater's task/level are not reachable from outside GripperServer
// (updater_ is private, as it should be), so the only testable seam is the same one the
// conformance suite uses: subscribe to /diagnostics and let the updater's own periodic
// timer (1 Hz) publish it. Slower than the other tests here, but real -- it exercises
// GripperServer::diagnostics() exactly as it runs in the node.
diagnostic_msgs::msg::DiagnosticStatus wait_for_gripper_task(
    rclcpp::Node::SharedPtr node, std::function<void(std::chrono::milliseconds)> spin_for) {
  diagnostic_msgs::msg::DiagnosticStatus got;
  bool seen = false;
  auto sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable(),
      [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {
        for (const auto& s : m->status) {
          if (s.name.find("Gripper") != std::string::npos) { got = s; seen = true; }
        }
      });
  for (int i = 0; i < 60 && !seen; ++i) spin_for(std::chrono::milliseconds(50));
  return seen ? got : throw std::runtime_error("no Gripper task seen on /diagnostics");
}
}  // namespace

TEST_F(GripperServerTest, DiagnosticsWarnsWhenGripperExpectedButAbsent) {
  FakeGripperSink sink;
  sink.state.present = false;
  kinova_gen3_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);
  (void)server;

  const auto got = wait_for_gripper_task(
      node_, [this](std::chrono::milliseconds d) { spin_for(d); });
  EXPECT_EQ(got.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(GripperServerTest, DiagnosticsOkWhenGripperNotExpectedAndAbsent) {
  FakeGripperSink sink;
  sink.state.present = false;
  kinova_gen3_ros2::GripperServer server(node_, sink, /*expect_gripper=*/false);
  (void)server;

  const auto got = wait_for_gripper_task(
      node_, [this](std::chrono::milliseconds d) { spin_for(d); });
  EXPECT_EQ(got.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

// The joint is published even when present == false. Omitting it would drop the
// gripper out of TF entirely -- its links would just vanish from the model rather than
// render at a frozen or default pose (verified 2026-09-03: robot_state_publisher
// publishes per-segment, not all-or-nothing for the whole robot; the arm's own TF is
// unaffected either way). /gripper_state.present is where absence is reported truthfully.
TEST_F(GripperServerTest, KnuckleJointIsPublishedEvenWhenAbsent) {
  FakeGripperSink sink;
  sink.state.position = 0.5f; sink.state.present = false;
  auto backend = std::make_shared<kinova_gen3_ros2::Ros2Backend>(node_);
  backend->set_gripper_sink(&sink);

  sensor_msgs::msg::JointState got;
  bool seen = false;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::SharedPtr m) { got = *m; seen = true; });
  spin_for(200ms);

  kinova::interface::ArmState s;
  for (int i = 0; i < 20 && !seen; ++i) { backend->publish_state(s); spin_for(20ms); }

  ASSERT_TRUE(seen);
  ASSERT_EQ(got.name.size(), 8u);
  EXPECT_EQ(got.name[7], "robotiq_85_left_knuckle_joint");
  EXPECT_DOUBLE_EQ(got.position[7], 0.4);          // 0.5 * 0.8
  EXPECT_TRUE(std::isnan(got.velocity[7]));        // core removed the field entirely
  EXPECT_TRUE(std::isnan(got.effort[7]));          // 0..1 fraction, not N*m
}

// Without a sink wired, the message is exactly what it was before this tier.
TEST_F(GripperServerTest, NoGripperSinkMeansSevenJointsAsBefore) {
  auto backend = std::make_shared<kinova_gen3_ros2::Ros2Backend>(node_);
  sensor_msgs::msg::JointState got;
  bool seen = false;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::SharedPtr m) { got = *m; seen = true; });
  spin_for(200ms);

  kinova::interface::ArmState s;
  for (int i = 0; i < 20 && !seen; ++i) { backend->publish_state(s); spin_for(20ms); }
  ASSERT_TRUE(seen);
  EXPECT_EQ(got.name.size(), 7u);
}
