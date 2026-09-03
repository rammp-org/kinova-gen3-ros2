#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "kinova_arm_ros2/gripper_server.h"
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"
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
  kinova_arm_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  auto pub = node_->create_publisher<kinova_arm_interfaces::msg::GripperSetpoint>(
      "/setpoint/gripper", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  spin_for(200ms);   // let discovery settle

  kinova_arm_interfaces::msg::GripperSetpoint m;
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
  kinova_arm_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  kinova_arm_interfaces::msg::GripperState got;
  bool seen = false;
  auto sub = node_->create_subscription<kinova_arm_interfaces::msg::GripperState>(
      "/gripper_state", rclcpp::SensorDataQoS(),
      [&](kinova_arm_interfaces::msg::GripperState::SharedPtr msg) { got = *msg; seen = true; });
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
