// ExecuteJointTrajectory goal validation in Ros2Backend::handle_goal.
#include <chrono>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_gen3_ros2/ros2_backend.h"
#include "planned_move_test_fixture.h"

using namespace std::chrono_literals;
using Action = rammp_arm_interfaces::action::ExecuteJointTrajectory;

namespace {
Action::Goal one_point_goal(double speed_scale) {
  Action::Goal g;
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.assign(kinova::kNumJoints, 0.0);
  p.time_from_start.sec = 1;
  g.trajectory.points.push_back(p);
  g.speed_scale = speed_scale;
  return g;
}
} // namespace

class Ros2BackendTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

// Returns true if the server accepted the goal.
static bool send(const rclcpp::Node::SharedPtr &node, double speed_scale) {
  auto client = rclcpp_action::create_client<Action>(node, "execute_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  if (!client->wait_for_action_server(5s))
    return false;
  auto fut = client->async_send_goal(one_point_goal(speed_scale));
  if (ex.spin_until_future_complete(fut, 5s) != rclcpp::FutureReturnCode::SUCCESS)
    return false;
  return fut.get() != nullptr;
}

TEST_F(Ros2BackendTest, ABadSpeedScaleIsRefusedBeforeTheSink) {
  auto node = std::make_shared<rclcpp::Node>("backend_speed_bad");
  kinova_gen3_ros2::Ros2Backend backend(node);
  FakeSupervisor sup(backend);
  backend.set_command_sink(&sup);
  for (double bad : {0.0, -1.0, 1.5}) {
    EXPECT_FALSE(send(node, bad)) << bad;
    EXPECT_FALSE(sup.got_goal) << bad;
  }
}

TEST_F(Ros2BackendTest, AValidSpeedScaleReachesTheSink) {
  auto node = std::make_shared<rclcpp::Node>("backend_speed_ok");
  kinova_gen3_ros2::Ros2Backend backend(node);
  FakeSupervisor sup(backend);
  backend.set_command_sink(&sup);
  EXPECT_TRUE(send(node, 0.5));
  EXPECT_TRUE(sup.got_goal);
  EXPECT_DOUBLE_EQ(sup.last_goal.speed_scale, 0.5);
}
