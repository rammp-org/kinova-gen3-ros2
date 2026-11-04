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
// record of what it received. Fails the test unless the plan succeeded and
// exactly one goal reached the fake, so `hold` is what was actually sent.
struct Seen {
  uint8_t hold;
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
  // Guard, don't assert-and-get: a never-set promise must fail the test, not
  // block the whole binary on get() until the CI timeout.
  if (f.wait_for(5s) != std::future_status::ready) {
    ADD_FAILURE() << name << ": on_done never fired";
    return {fake.last_hold()};
  }
  EXPECT_TRUE(f.get().ok);
  EXPECT_EQ(fake.pose_goals_received(), 1);
  return {fake.last_hold()};
}
} // namespace

TEST_F(CuroboClientTest, TranslatesEveryHoldModeOntoThePlannerContract) {
  // The translation is a switch over three constants that currently happen to
  // share numeric values with the planner's. Asserting only one mode would
  // pass even if two cases were transposed, so every mode is exercised and
  // compared against the PLANNER's own constant rather than a literal.
  using Arm = rammp_arm_interfaces::action::GoToEEPose::Goal;
  using Planner = rammp_curobo_interfaces::action::PlanToPose::Goal;
  const std::pair<uint8_t, uint8_t> cases[] = {
      {Arm::HOLD_NONE, Planner::HOLD_NONE},
      {Arm::HOLD_LEVEL, Planner::HOLD_LEVEL},
      {Arm::HOLD_FIXED, Planner::HOLD_FIXED},
  };
  for (const auto &[arm, planner] : cases) {
    const auto seen = run_plan(("hold_test" + std::to_string(arm)).c_str(),
                               [&](CuroboPlanClient &c, auto done) {
                                 c.plan(geometry_msgs::msg::Pose{},
                                        kStartJoints, arm, nullptr, done);
                               });
    EXPECT_EQ(seen.hold, planner) << "arm mode " << int(arm);
  }
}

namespace {
// Plans with a malformed hold against a live fake and returns the
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
  // Guard, don't assert-and-get: a never-set promise must fail the test, not
  // block the whole binary on get() until the CI timeout.
  if (f.wait_for(5s) != std::future_status::ready) {
    ADD_FAILURE() << name << ": on_done never fired";
    CuroboPlanClient::Outcome none;
    return {std::move(none), fake.pose_goals_received()};
  }
  // Give a wrongly dispatched goal time to reach the server.
  std::this_thread::sleep_for(300ms);
  return {f.get(), fake.pose_goals_received()};
}
} // namespace

TEST_F(CuroboClientTest, AnUnknownHoldModeFailsLoudInsteadOfPlanning) {
  // Asserting `received == 0` is the real content: the failure that matters is
  // not a bad error string, it is dispatching an UNCONSTRAINED plan for a
  // caller who asked for a held one.
  const uint8_t hold = 9;
  const auto [o, received] =
      run_refused("bad_hold_test", [&](CuroboPlanClient &c, auto done) {
        c.plan(geometry_msgs::msg::Pose{}, kStartJoints, hold, nullptr, done);
      });
  EXPECT_FALSE(o.ok);
  EXPECT_NE(o.message.find("9"), std::string::npos);
  EXPECT_EQ(received, 0);
}

TEST_F(CuroboClientTest, TheShortPlanOverloadSendsNoHold) {
  // The overload without a hold must reach the planner as HOLD_NONE, not as
  // whatever an uninitialised field happens to contain.
  const auto seen =
      run_plan("short_overload_test", [&](CuroboPlanClient &c, auto done) {
        c.plan(geometry_msgs::msg::Pose{}, kStartJoints, nullptr, done);
      });
  EXPECT_EQ(seen.hold,
            rammp_curobo_interfaces::action::PlanToPose::Goal::HOLD_NONE);
}

namespace {
// Two 3-point legs, rest to rest, the second starting where the first ends.
std::vector<trajectory_msgs::msg::JointTrajectory> two_legs() {
  std::vector<trajectory_msgs::msg::JointTrajectory> legs(2);
  for (size_t l = 0; l < legs.size(); ++l)
    for (int k = 0; k < 3; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(7, 0.1 * (2 * l + k));
      p.velocities.assign(7, k == 1 ? 0.5 : 0.0);
      legs[l].points.push_back(p);
    }
  return legs;
}

geometry_msgs::msg::Quaternion quat(double x, double y, double z, double w) {
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

// Blends `legs` against a live fake; returns the outcome and the fake's view.
struct Blended {
  CuroboPlanClient::Outcome outcome;
  CuroboPlanClient::Blend::Goal seen;
  int received;
};
Blended
run_blend(const char *name,
          const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
          const std::vector<double> &radii, const std::vector<double> &scales,
          const std::vector<uint8_t> &holds,
          const std::vector<geometry_msgs::msg::Quaternion> &orientations) {
  auto node = std::make_shared<rclcpp::Node>(name);
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);
  // Shared with the callback, so an outcome arriving after the timeout below
  // lands in a live promise rather than a destroyed one.
  auto p = std::make_shared<std::promise<CuroboPlanClient::Outcome>>();
  auto f = p->get_future();
  client.blend(
      legs, radii, scales, holds, orientations, nullptr,
      [p](CuroboPlanClient::Outcome o) { p->set_value(std::move(o)); });
  if (f.wait_for(5s) != std::future_status::ready) {
    ADD_FAILURE() << name << ": on_done never fired";
    return {{}, fake.last_blend(), fake.blend_goals_received()};
  }
  auto o = f.get();
  std::this_thread::sleep_for(100ms); // let a wrongly sent goal land
  return {std::move(o), fake.last_blend(), fake.blend_goals_received()};
}
} // namespace

TEST_F(CuroboClientTest, BlendCarriesTheLegsRadiiScalesHoldsAndOrientations) {
  using Arm = rammp_arm_interfaces::action::GoToEEPose::Goal;
  using Planner = rammp_curobo_interfaces::action::PlanToPose::Goal;
  const auto b =
      run_blend("blend_test", two_legs(), {0.03}, {0.8, 0.4},
                {Arm::HOLD_LEVEL, Arm::HOLD_FIXED},
                {quat(0.0, 0.0, 0.0, 1.0), quat(1.0, 0.0, 0.0, 0.0)});
  EXPECT_TRUE(b.outcome.ok) << b.outcome.message;
  EXPECT_EQ(b.outcome.trajectory.points.size(), 6u); // the fake joins the legs
  ASSERT_EQ(b.seen.legs.size(), 2u);
  EXPECT_EQ(b.seen.legs[1].points[0].positions,
            two_legs()[1].points[0].positions);
  EXPECT_EQ(b.seen.blend_radius, std::vector<double>({0.03}));
  EXPECT_EQ(b.seen.speed_scale, std::vector<double>({0.8, 0.4}));
  EXPECT_EQ(b.seen.hold,
            std::vector<uint8_t>({Planner::HOLD_LEVEL, Planner::HOLD_FIXED}));
  ASSERT_EQ(b.seen.hold_orientation.size(), 2u);
  EXPECT_EQ(b.seen.hold_orientation[1], quat(1.0, 0.0, 0.0, 0.0));
}

TEST_F(CuroboClientTest, AnUnknownBlendHoldFailsLoudInsteadOfBlending) {
  const auto b =
      run_blend("blend_bad_hold_test", two_legs(), {0.0}, {}, {0, 9}, {});
  EXPECT_FALSE(b.outcome.ok);
  EXPECT_NE(b.outcome.message.find("9"), std::string::npos);
  EXPECT_EQ(b.received, 0);
}

// The fake blend is what the GoTo tests chain legs against, so the refusals
// it does model must really fire; otherwise a chaining bug would pass them.
// It models legs that do not join (1e-5 rad), points without velocities,
// one-point legs and wrong array lengths -- not the rest of the real blend's.
TEST_F(CuroboClientTest, TheFakeBlendRefusesBadJoinsVelocitiesAndShortLegs) {
  auto gap = two_legs();
  gap[1].points[0].positions[3] += 0.002;
  auto no_velocities = two_legs();
  no_velocities[0].points[1].velocities.clear();
  auto one_point = two_legs();
  one_point[1].points.resize(1);
  const std::pair<const char *,
                  std::vector<trajectory_msgs::msg::JointTrajectory>>
      cases[] = {{"legs that do not join", gap},
                 {"a point without velocities", no_velocities},
                 {"a one-point leg", one_point}};
  for (const auto &[what, legs] : cases) {
    const auto b = run_blend("fake_blend_refuses", legs, {0.0}, {}, {}, {});
    EXPECT_FALSE(b.outcome.ok) << what;
    EXPECT_NE(b.outcome.message.find("leg"), std::string::npos)
        << what << ": " << b.outcome.message;
  }
  EXPECT_FALSE(
      run_blend("fake_blend_radii", two_legs(), {}, {}, {}, {}).outcome.ok)
      << "2 legs, no radius";
}
