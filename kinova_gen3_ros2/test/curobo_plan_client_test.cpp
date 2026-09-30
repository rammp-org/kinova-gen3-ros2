#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kinova_gen3_ros2/curobo_plan_client.h"
#include "fake_curobo_server.h"
using namespace std::chrono_literals;
using kinova_gen3_ros2::CuroboPlanClient;

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
// The configuration a plan starts from. The caller always states it; an
// empty vector would tell cuRobo to source the arm state itself.
const std::vector<double> kStartJoints{0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57};

class SpinThread {
public:
  explicit SpinThread(rclcpp::Executor &ex)
      : ex_(ex), t_([&ex] { ex.spin(); }) {}
  ~SpinThread() {
    ex_.cancel();
    if (t_.joinable())
      t_.join();
  }
  SpinThread(const SpinThread &) = delete;
  SpinThread &operator=(const SpinThread &) = delete;

private:
  rclcpp::Executor &ex_;
  std::thread t_;
};
} // namespace

TEST_F(CuroboClientTest, PlanSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);
}

TEST_F(CuroboClientTest, PlanAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test2");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanRejectedReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test3");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3,
                                                /*reject=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr,
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
  client.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

// --- plan_to_joints: mirrors the four plan() cases above. FakeCuroboServer
// --- hosts both tiers off one configuration, so only the call differs.
namespace {
const std::vector<double> kTargetJoints = {0.0, 0.262, 3.142, -2.269,
                                           0.0, 0.96,  1.571};
} // namespace

TEST_F(CuroboClientTest, PlanToJointsSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test1");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(
      kTargetJoints, kStartJoints, nullptr,
      [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);
  EXPECT_DOUBLE_EQ(o.goal_mismatch_rad, 0.0);
}

TEST_F(CuroboClientTest, PlanToJointsAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test2");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(
      kTargetJoints, kStartJoints, nullptr,
      [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

TEST_F(CuroboClientTest, PlanToJointsRejectedReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_joints_test3");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3,
                                                /*reject=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan_to_joints(
      kTargetJoints, kStartJoints, nullptr,
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
  client.plan_to_joints(
      kTargetJoints, kStartJoints, nullptr,
      [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());
}

namespace {
// Runs one pose plan to completion against a fake and hands back the fake's
// record of what it received.
struct Seen {
  rammp_curobo_interfaces::msg::PoseAxisLock lock;
  rammp_curobo_interfaces::msg::ApproachVia via;
};
template <typename PlanFn> Seen run_plan(const char *name, PlanFn call) {
  auto node = std::make_shared<rclcpp::Node>(name);
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);
  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  call(client, [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  EXPECT_EQ(f.wait_for(5s), std::future_status::ready);
  return {fake.last_axis_lock(), fake.last_approach_via()};
}
} // namespace

TEST_F(CuroboClientTest, TranslatesTheArmLockOntoThePlannerLock) {
  // A mis-mapped boolean constrains the WRONG axis and still plans, so assert
  // all six and the frame rather than "some lock arrived".
  rammp_arm_interfaces::msg::ToolAxisLock lock;
  lock.lock_roll = true;
  lock.lock_z = true;
  lock.reference_frame = rammp_arm_interfaces::msg::ToolAxisLock::FRAME_BASE;
  const auto seen = run_plan("lock_test", [&](CuroboPlanClient &c, auto done) {
    c.plan(geometry_msgs::msg::Pose{}, kStartJoints, lock,
           rammp_arm_interfaces::msg::ApproachOffset{}, nullptr, done);
  });
  EXPECT_TRUE(seen.lock.lock_roll);
  EXPECT_FALSE(seen.lock.lock_pitch);
  EXPECT_FALSE(seen.lock.lock_yaw);
  EXPECT_FALSE(seen.lock.lock_x);
  EXPECT_FALSE(seen.lock.lock_y);
  EXPECT_TRUE(seen.lock.lock_z);
  EXPECT_EQ(seen.lock.reference_frame,
            rammp_curobo_interfaces::msg::PoseAxisLock::FRAME_BASE);
}

TEST_F(CuroboClientTest, TranslatesTheGoalFrame) {
  rammp_arm_interfaces::msg::ToolAxisLock lock;
  lock.lock_pitch = true;
  lock.lock_x = true;
  lock.reference_frame = rammp_arm_interfaces::msg::ToolAxisLock::FRAME_GOAL;
  const auto seen = run_plan("frame_test", [&](CuroboPlanClient &c, auto done) {
    c.plan(geometry_msgs::msg::Pose{}, kStartJoints, lock,
           rammp_arm_interfaces::msg::ApproachOffset{}, nullptr, done);
  });
  EXPECT_FALSE(seen.lock.lock_roll);
  EXPECT_TRUE(seen.lock.lock_pitch);
  EXPECT_FALSE(seen.lock.lock_yaw);
  EXPECT_TRUE(seen.lock.lock_x);
  EXPECT_FALSE(seen.lock.lock_y);
  EXPECT_FALSE(seen.lock.lock_z);
  EXPECT_EQ(seen.lock.reference_frame,
            rammp_curobo_interfaces::msg::PoseAxisLock::FRAME_GOAL);
}

TEST_F(CuroboClientTest, TranslatesTheApproachOffset) {
  rammp_arm_interfaces::msg::ApproachOffset off;
  off.distance = 0.10;
  off.axis = rammp_arm_interfaces::msg::ApproachOffset::AXIS_Y;
  off.at_fraction = 0.7;
  const auto seen = run_plan("via_test", [&](CuroboPlanClient &c, auto done) {
    c.plan(geometry_msgs::msg::Pose{}, kStartJoints,
           rammp_arm_interfaces::msg::ToolAxisLock{}, off, nullptr, done);
  });
  EXPECT_DOUBLE_EQ(seen.via.offset, 0.10);
  EXPECT_EQ(seen.via.axis, rammp_curobo_interfaces::msg::ApproachVia::AXIS_Y);
  EXPECT_DOUBLE_EQ(seen.via.at_fraction, 0.7);
}

namespace {
// Plans with a malformed lock/offset against a live fake and returns the
// outcome plus how many goals the fake received (must be zero).
template <typename PlanFn>
std::pair<CuroboPlanClient::Outcome, int> run_refused(const char *name,
                                                      PlanFn call) {
  auto node = std::make_shared<rclcpp::Node>(name);
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);
  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  call(client, [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  EXPECT_EQ(f.wait_for(5s), std::future_status::ready);
  // Give a wrongly dispatched goal time to reach the server.
  std::this_thread::sleep_for(300ms);
  return {f.get(), fake.pose_goals_received()};
}
} // namespace

TEST_F(CuroboClientTest, AnUnknownFrameFailsLoudInsteadOfPlanning) {
  rammp_arm_interfaces::msg::ToolAxisLock lock;
  lock.reference_frame = 9;
  const auto [o, received] =
      run_refused("bad_frame_test", [&](CuroboPlanClient &c, auto done) {
        c.plan(geometry_msgs::msg::Pose{}, kStartJoints, lock,
               rammp_arm_interfaces::msg::ApproachOffset{}, nullptr, done);
      });
  EXPECT_FALSE(o.ok);
  EXPECT_NE(o.message.find("9"), std::string::npos);
  EXPECT_EQ(received, 0);
}

TEST_F(CuroboClientTest, AnUnknownAxisFailsLoudInsteadOfPlanning) {
  rammp_arm_interfaces::msg::ApproachOffset off;
  off.distance = 0.1;
  off.axis = 7;
  const auto [o, received] =
      run_refused("bad_axis_test", [&](CuroboPlanClient &c, auto done) {
        c.plan(geometry_msgs::msg::Pose{}, kStartJoints,
               rammp_arm_interfaces::msg::ToolAxisLock{}, off, nullptr, done);
      });
  EXPECT_FALSE(o.ok);
  EXPECT_NE(o.message.find("7"), std::string::npos);
  EXPECT_EQ(received, 0);
}

TEST_F(CuroboClientTest, TheFourArgumentPlanSendsNothingExtra) {
  const auto seen =
      run_plan("four_arg_test", [&](CuroboPlanClient &c, auto done) {
        c.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr, done);
      });
  EXPECT_FALSE(seen.lock.lock_roll || seen.lock.lock_pitch ||
               seen.lock.lock_yaw || seen.lock.lock_x || seen.lock.lock_y ||
               seen.lock.lock_z);
  EXPECT_DOUBLE_EQ(seen.via.offset, 0.0);
  EXPECT_EQ(seen.lock.reference_frame,
            rammp_curobo_interfaces::msg::PoseAxisLock::FRAME_BASE);
  EXPECT_EQ(seen.via.axis, rammp_curobo_interfaces::msg::ApproachVia::AXIS_Z);
  EXPECT_DOUBLE_EQ(seen.via.at_fraction, 0.8);
}
