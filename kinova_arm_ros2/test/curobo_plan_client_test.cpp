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

namespace {
// Spins an executor on a background thread and always cancels + joins it.
// A bare `std::thread spin(...)` joined at the end of the test body is skipped
// whenever an ASSERT_* returns early, and destroying a joinable thread calls
// std::terminate — turning a legible gtest failure into a bare SIGABRT.
class SpinThread {
 public:
  explicit SpinThread(rclcpp::Executor& ex) : ex_(ex), t_([&ex] { ex.spin(); }) {}
  ~SpinThread() { ex_.cancel(); if (t_.joinable()) t_.join(); }
  SpinThread(const SpinThread&) = delete;
  SpinThread& operator=(const SpinThread&) = delete;

 private:
  rclcpp::Executor& ex_;
  std::thread t_;
};
}  // namespace

TEST_F(CuroboClientTest, PlanSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);
}

TEST_F(CuroboClientTest, PlanAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanRejectedReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test3");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3,
                                                /*reject=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanServerUnavailableReturnsFailure) {
  // No FakeCuroboServer constructed. Point the client at a name nothing ever
  // serves rather than relying on the default being unserved: the preceding
  // tests' fake servers linger in DDS discovery long enough that the default
  // name is sometimes still matched here, in which case the goal is sent into
  // the void and no result callback ever arrives (a 5 s hang, not a failure).
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test4");
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp, "/rammp_curobo/plan_to_pose_unserved");

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

// --- plan_to_joints: mirrors the four plan() cases above. FakeCuroboServer
// --- hosts both tiers off one configuration, so only the call differs.
namespace {
const std::vector<double> kTargetJoints = {0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571};
}  // namespace

TEST_F(CuroboClientTest, PlanToJointsSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test1");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(kTargetJoints, nullptr,
                        [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);
  EXPECT_DOUBLE_EQ(o.goal_mismatch_rad, 0.0);
}

TEST_F(CuroboClientTest, PlanToJointsAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(kTargetJoints, nullptr,
                        [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanToJointsRejectedReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test3");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3,
                                                /*reject=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(kTargetJoints, nullptr,
                        [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanToJointsServerUnavailableReturnsFailure) {
  // As for the pose tier: name a joints action nothing ever serves, so stale
  // discovery from earlier tests cannot make this hang instead of failing.
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test4");
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp, "/rammp_curobo/plan_to_pose_unserved",
                          "/rammp_curobo/plan_to_joints_unserved");

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(kTargetJoints, nullptr,
                        [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}
