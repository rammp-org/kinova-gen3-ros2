#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "fake_curobo_server.h"
using namespace std::chrono_literals;
using kinova_arm_ros2::CuroboPlanClient;

class CuroboClientTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(CuroboClientTest, PlanSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);

  ex.cancel();
  spin.join();
}

TEST_F(CuroboClientTest, PlanAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());

  ex.cancel();
  spin.join();
}

TEST_F(CuroboClientTest, PlanRejectedReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test3");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3,
                                                /*reject=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());

  ex.cancel();
  spin.join();
}

TEST_F(CuroboClientTest, PlanServerUnavailableReturnsFailure) {
  // No FakeCuroboServer constructed -> no server behind the default action name.
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test4");
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());

  ex.cancel();
  spin.join();
}
