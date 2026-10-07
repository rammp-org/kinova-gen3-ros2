// ExecuteJointTrajectory goal validation in Ros2Backend::handle_goal.
#include <chrono>
#include <cmath>
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

// Returns true if the server accepted the goal. A missing server or a goal
// response that never came FAILS the test rather than returning false, so
// "refused" cannot be satisfied by a dead server.
static bool send(const rclcpp::Node::SharedPtr &node, double speed_scale) {
  auto client =
      rclcpp_action::create_client<Action>(node, "execute_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  if (!client->wait_for_action_server(5s)) {
    ADD_FAILURE() << "execute_joint_trajectory server never appeared";
    return false;
  }
  auto fut = client->async_send_goal(one_point_goal(speed_scale));
  if (ex.spin_until_future_complete(fut, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    ADD_FAILURE() << "no goal response within 5 s";
    return false;
  }
  return fut.get() != nullptr;
}

TEST_F(Ros2BackendTest, ABadSpeedScaleIsRefusedBeforeTheSink) {
  auto node = std::make_shared<rclcpp::Node>("backend_speed_bad");
  kinova_gen3_ros2::Ros2Backend backend(node);
  FakeSupervisor sup(backend);
  backend.set_command_sink(&sup);
  for (double bad : {0.0, -1.0, 1.5, std::nan("")}) {
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

// Gains on a POSITION goal cannot act; refused before the sink, with the
// reason in the node log (the driver would refuse it silently).
TEST_F(Ros2BackendTest, GainsOnAPositionGoalAreRefusedBeforeTheSink) {
  auto node = std::make_shared<rclcpp::Node>("backend_gains_bad");
  kinova_gen3_ros2::Ros2Backend backend(node);
  FakeSupervisor sup(backend);
  backend.set_command_sink(&sup);
  auto goal = one_point_goal(1.0);
  goal.control_mode = 0;
  goal.gains.profile = rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_STIFF;

  auto client =
      rclcpp_action::create_client<Action>(node, "execute_joint_trajectory");
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ASSERT_TRUE(client->wait_for_action_server(5s));
  auto fut = client->async_send_goal(goal);
  ASSERT_EQ(ex.spin_until_future_complete(fut, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(fut.get(), nullptr);
  EXPECT_FALSE(sup.got_goal);
}

// /set_gains: local validation refuses what cannot be mapped; everything else
// is relayed to the CommandSink (the Arbiter, in bringup) and the sink's
// verdict comes back verbatim.
TEST_F(Ros2BackendTest, SetGainsMapsAndRelaysTheSinkVerdict) {
  using Srv = rammp_arm_interfaces::srv::SetGains;
  auto node = std::make_shared<rclcpp::Node>("backend_set_gains");
  kinova_gen3_ros2::Ros2Backend backend(node);
  FakeSupervisor sup(backend);
  backend.set_command_sink(&sup);

  auto client = node->create_client<Srv>("set_gains");
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ASSERT_TRUE(client->wait_for_service(5s));

  auto req = std::make_shared<Srv::Request>();
  req->spec.profile = rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_SOFT;
  req->token.fill(0);
  req->token[0] = 0xEE;
  auto fut = client->async_send_request(req);
  ASSERT_EQ(ex.spin_until_future_complete(fut, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut.get()->accepted);
  ASSERT_TRUE(sup.got_gains);
  EXPECT_EQ(sup.last_gains.spec.profile,
            kinova::interface::GainsProfile::kSoft);
  EXPECT_EQ(sup.last_gains.token[0], 0xEE); // the capability survives the hop

  // An unknown profile byte never reaches the sink as "session default".
  sup.got_gains = false;
  auto bad = std::make_shared<Srv::Request>();
  bad->spec.profile = 9;
  auto fut2 = client->async_send_request(bad);
  ASSERT_EQ(ex.spin_until_future_complete(fut2, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(fut2.get()->accepted);
  EXPECT_FALSE(sup.got_gains);

  // The sink's refusal is relayed with its message.
  sup.gains_result = {false, "refused by fake"};
  auto fut3 = client->async_send_request(req);
  ASSERT_EQ(ex.spin_until_future_complete(fut3, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  // get() once: an rclcpp client future's shared state is CONSUMED by get(),
  // and a second call throws std::future_error (no associated state).
  const auto resp3 = fut3.get();
  EXPECT_FALSE(resp3->accepted);
  EXPECT_EQ(resp3->message, "refused by fake");
}
