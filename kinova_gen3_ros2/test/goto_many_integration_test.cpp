#include <gtest/gtest.h>
#include <chrono>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_poses.hpp"
#include "rammp_arm_interfaces/action/go_to_joint_configs.hpp"
#include "kinova_gen3_ros2/curobo_plan_client.h"
#include "kinova_gen3_ros2/goal_router.h"
#include "kinova_gen3_ros2/goto_ee_poses_server.h"
#include "kinova_gen3_ros2/goto_joint_configs_server.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_curobo_server.h"
#include "planned_move_test_fixture.h"
using namespace std::chrono_literals;
using namespace kinova::interface;
using GoToEEPoses = rammp_arm_interfaces::action::GoToEEPoses;
using GoToJointConfigs = rammp_arm_interfaces::action::GoToJointConfigs;
using PlannerHold = rammp_curobo_interfaces::action::PlanToPose::Goal;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Sent {
  int code;
  std::string error;
};

// Send a goal and block for its result; kGoalRejected if not accepted.
template <class ActionT>
Sent send(rclcpp::Node::SharedPtr node, const std::string &name,
          const typename ActionT::Goal &goal) {
  auto client = rclcpp_action::create_client<ActionT>(node, name);
  if (!client->wait_for_action_server(5s))
    return {kServerMissing, ""};
  std::promise<Sent> result;
  auto fut = result.get_future();
  typename rclcpp_action::Client<ActionT>::SendGoalOptions opts;
  opts.result_callback =
      [&](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult
              &wr) {
        result.set_value(
            wr.result ? Sent{wr.result->error_code, wr.result->error_string}
                      : Sent{kNoResult, ""});
      };
  auto gh_future = client->async_send_goal(goal, opts);
  if (gh_future.wait_for(5s) != std::future_status::ready)
    return {kTimedOut, ""};
  if (gh_future.get() == nullptr)
    return {kGoalRejected, ""};
  if (fut.wait_for(8s) != std::future_status::ready)
    return {kTimedOut, ""};
  return fut.get();
}

// One server against the fake planner and a supervisor that "executes" each
// accepted trajectory at once. Declared in dependency order, so the spin
// thread stops first.
template <class ServerT> struct Rig {
  rclcpp::Node::SharedPtr node;
  kinova_gen3_ros2::test::FakeCuroboServer fake;
  rclcpp::CallbackGroup::SharedPtr grp;
  kinova_gen3_ros2::CuroboPlanClient planner;
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router;
  ServerT server;
  FakeSupervisor sup;
  rclcpp::executors::MultiThreadedExecutor ex;
  std::unique_ptr<SpinThread> spin;
  explicit Rig(const char *name)
      : node(std::make_shared<rclcpp::Node>(name)),
        fake(node, /*succeed=*/true, /*n_points=*/3),
        grp(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
        planner(node, grp), router(dummy), server(node, router, planner, grp),
        sup(router) {
    server.set_command_sink(&sup);
    ex.add_node(node);
    spin = std::make_unique<SpinThread>(ex);
  }
};
using PosesRig = Rig<kinova_gen3_ros2::GoToEEPosesServer>;
using ConfigsRig = Rig<kinova_gen3_ros2::GoToJointConfigsServer>;

GoToEEPoses::Goal poses_goal(size_t n) {
  GoToEEPoses::Goal g;
  for (size_t k = 0; k < n; ++k) {
    geometry_msgs::msg::PoseStamped t;
    t.header.frame_id = "arm_mount";
    t.pose.position.x = 0.4;
    t.pose.position.y = 0.1 * static_cast<double>(k);
    t.pose.position.z = 0.3;
    t.pose.orientation.w = 1.0;
    g.targets.push_back(t);
  }
  return g;
}

GoToJointConfigs::Goal configs_goal(size_t n) {
  GoToJointConfigs::Goal g;
  for (size_t k = 0; k < n; ++k) {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {
        0.1 * static_cast<double>(k), 0.262, 3.142, -2.269, 0.0, 0.96, 1.571};
    g.targets.push_back(p);
  }
  return g;
}

// Each leg after the first must be planned from the last point of the leg the
// planner returned before it, and the first from the measured state (zero in
// FakeSupervisor).
void expect_chained(const kinova_gen3_ros2::test::FakeCuroboServer &fake,
                    size_t n) {
  const auto starts = fake.starts_seen();
  const auto legs = fake.last_blend().legs;
  ASSERT_EQ(starts.size(), n);
  ASSERT_EQ(legs.size(), n);
  EXPECT_EQ(starts[0], std::vector<double>(7, 0.0));
  for (size_t k = 1; k < n; ++k) {
    EXPECT_EQ(starts[k], legs[k - 1].points.back().positions) << "leg " << k;
    EXPECT_NE(starts[k], starts[k - 1]) << "leg " << k;
  }
}
} // namespace

class GotoManyTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// --- GoToEEPoses ------------------------------------------------------------

TEST_F(GotoManyTest, PosesRefusesMalformedGoalsAtSubmission) {
  PosesRig rig("goto_poses_refuse");
  using G = GoToEEPoses::Goal;
  const std::vector<std::pair<const char *, std::function<void(G &)>>> cases = {
      {"no targets", [](G &g) { g.targets.clear(); }},
      {"base_link target",
       [](G &g) { g.targets[1].header.frame_id = "base_link"; }},
      {"empty frame", [](G &g) { g.targets[2].header.frame_id = ""; }},
      {"2 speeds for 3",
       [](G &g) {
         g.speed_scale = {0.5, 0.5};
       }},
      {"zero speed", [](G &g) { g.speed_scale = {0.0}; }},
      {"fast leg",
       [](G &g) {
         g.speed_scale = {1.0, 1.5, 1.0};
       }},
      {"NaN speed", [](G &g) { g.speed_scale = {kNaN}; }},
      {"unknown hold", [](G &g) { g.orientation_hold = {7}; }},
      {"unknown hold on one leg",
       [](G &g) {
         g.orientation_hold = {0, 1, 3};
       }},
      {"2 holds for 3",
       [](G &g) {
         g.orientation_hold = {0, 0};
       }},
      {"1 radius for 2 junctions", [](G &g) { g.blend_radius = {0.01}; }},
      {"3 radii for 2 junctions",
       [](G &g) {
         g.blend_radius = {0.01, 0.01, 0.01};
       }},
      {"negative radius",
       [](G &g) {
         g.blend_radius = {-0.01, 0.0};
       }},
      {"NaN radius",
       [](G &g) {
         g.blend_radius = {0.0, kNaN};
       }},
      {"infinite radius",
       [](G &g) {
         g.blend_radius = {std::numeric_limits<double>::infinity(), 0.0};
       }},
      {"gains on a position goal",
       [](G &g) {
         g.gains.profile =
             rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_STIFF;
       }},
      {"unknown control_mode", [](G &g) { g.control_mode = 5; }},
  };
  for (const auto &[what, mutate] : cases) {
    auto g = poses_goal(3);
    mutate(g);
    EXPECT_EQ(send<GoToEEPoses>(rig.node, "go_to_ee_poses", g).code,
              kGoalRejected)
        << what;
  }
  EXPECT_EQ(rig.fake.pose_goals_received(), 0) << "a refused goal was planned";
  EXPECT_EQ(rig.fake.blend_goals_received(), 0);
  EXPECT_FALSE(rig.sup.got_goal);
}

TEST_F(GotoManyTest, PosesLegsChainFromThePreviousLegsLastPoint) {
  PosesRig rig("goto_poses_chain");
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(3));
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  expect_chained(rig.fake, 3);
  // What executes is the blend's trajectory, not any one leg.
  ASSERT_TRUE(rig.sup.got_goal);
  EXPECT_EQ(rig.sup.last_goal.trajectory.points.size(), 9u);
}

TEST_F(GotoManyTest, PosesAFailingLegIsNamedAndNothingMoves) {
  PosesRig rig("goto_poses_leg_fails");
  rig.fake.set_fail_plan_number(2);
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(3));
  EXPECT_EQ(r.code, result_code::kPlanningFailed);
  EXPECT_EQ(r.error, "leg 2/3: fake planner: no solution");
  EXPECT_EQ(rig.fake.starts_seen().size(), 2u) << "leg 3 must not be planned";
  EXPECT_EQ(rig.fake.blend_goals_received(), 0);
  EXPECT_FALSE(rig.sup.got_goal) << "a failed leg must never reach the arm";
}

TEST_F(GotoManyTest, PosesAFailingBlendFailsTheGoalAndNothingMoves) {
  PosesRig rig("goto_poses_blend_fails");
  rig.fake.set_blend_succeed(false);
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(2));
  EXPECT_EQ(r.code, result_code::kPlanningFailed);
  EXPECT_EQ(r.error, "blend: fake blend: no overlap");
  EXPECT_FALSE(rig.sup.got_goal);
}

// The blend carries the per-leg speeds; scaling it again at execution would
// apply them twice.
TEST_F(GotoManyTest, PosesTheBlendExecutesAtSpeedScaleOne) {
  PosesRig rig("goto_poses_speed");
  auto g = poses_goal(2);
  g.speed_scale = {0.5, 0.3};
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  EXPECT_EQ(rig.fake.last_blend().speed_scale, std::vector<double>({0.5, 0.3}));
  EXPECT_DOUBLE_EQ(rig.sup.last_goal.speed_scale, 1.0);
}

TEST_F(GotoManyTest, PosesTheBlendRequestCarriesRadiiScalesAndHolds) {
  PosesRig rig("goto_poses_blend_goal");
  auto g = poses_goal(3);
  g.speed_scale = {0.4}; // one value: every leg
  g.orientation_hold = {GoToEEPoses::Goal::HOLD_LEVEL,
                        GoToEEPoses::Goal::HOLD_NONE,
                        GoToEEPoses::Goal::HOLD_FIXED};
  g.blend_radius = {0.03, 0.0};
  auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.03, 0.0}));
  EXPECT_EQ(seen.speed_scale, std::vector<double>({0.4, 0.4, 0.4}));
  EXPECT_EQ(seen.hold, std::vector<uint8_t>({PlannerHold::HOLD_LEVEL,
                                             PlannerHold::HOLD_NONE,
                                             PlannerHold::HOLD_FIXED}));
  // ...and each leg is PLANNED with its hold too (the last plan: leg 3).
  EXPECT_EQ(rig.fake.last_hold(), PlannerHold::HOLD_FIXED);

  // Empty arrays: stop on every target, full speed, no holds.
  r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(3));
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  seen = rig.fake.last_blend();
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.0, 0.0}));
  EXPECT_EQ(seen.speed_scale, std::vector<double>({1.0, 1.0, 1.0}));
  EXPECT_EQ(seen.hold, std::vector<uint8_t>(3, PlannerHold::HOLD_NONE));
}

TEST_F(GotoManyTest, PosesOneTargetIsOneLegWithNoJunction) {
  PosesRig rig("goto_poses_one");
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(1));
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  EXPECT_EQ(rig.fake.last_blend().legs.size(), 1u);
  EXPECT_TRUE(rig.fake.last_blend().blend_radius.empty());
  EXPECT_TRUE(rig.sup.got_goal);
}

// The same post-plan check as GoToEEPose, on the trajectory that executes.
TEST_F(GotoManyTest, PosesABlendNotInArmMountIsRefused) {
  PosesRig rig("goto_poses_frame");
  rig.fake.set_frame_id("");
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(2));
  EXPECT_EQ(r.code, result_code::kPlanningFailed);
  EXPECT_NE(r.error.find("not arm_mount"), std::string::npos) << r.error;
  EXPECT_FALSE(rig.sup.got_goal);
}

// Impedance is one mode for the whole motion, executed as GoToEEPose's.
TEST_F(GotoManyTest, PosesImpedanceModeAndGainsReachTheTrajectoryGoal) {
  PosesRig rig("goto_poses_imp");
  auto g = poses_goal(2);
  g.control_mode = GoToEEPoses::Goal::CONTROL_MODE_IMPEDANCE;
  g.gains.profile = rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_SOFT;
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  EXPECT_EQ(rig.sup.last_goal.control_mode, ControlModeKind::kImpedance);
  EXPECT_EQ(rig.sup.last_goal.gains.profile, GainsProfile::kSoft);
}

// --- GoToJointConfigs -------------------------------------------------------

TEST_F(GotoManyTest, ConfigsRefusesMalformedGoalsAtSubmission) {
  ConfigsRig rig("goto_configs_refuse");
  using G = GoToJointConfigs::Goal;
  const std::vector<std::pair<const char *, std::function<void(G &)>>> cases = {
      {"no targets", [](G &g) { g.targets.clear(); }},
      {"6 positions", [](G &g) { g.targets[1].positions.pop_back(); }},
      {"8 positions", [](G &g) { g.targets[0].positions.push_back(0.0); }},
      {"NaN position", [](G &g) { g.targets[2].positions[3] = kNaN; }},
      {"velocities", [](G &g) { g.targets[0].velocities.assign(7, 0.0); }},
      {"accelerations",
       [](G &g) { g.targets[1].accelerations.assign(7, 0.0); }},
      {"effort", [](G &g) { g.targets[2].effort.assign(7, 0.0); }},
      {"time_from_start", [](G &g) { g.targets[0].time_from_start.sec = 1; }},
      {"time_from_start ns",
       [](G &g) { g.targets[0].time_from_start.nanosec = 1; }},
      {"2 speeds for 3",
       [](G &g) {
         g.speed_scale = {0.5, 0.5};
       }},
      {"zero speed", [](G &g) { g.speed_scale = {0.0}; }},
      {"1 radius for 2 junctions", [](G &g) { g.blend_radius = {0.01}; }},
      {"negative radius",
       [](G &g) {
         g.blend_radius = {0.0, -1.0};
       }},
      {"gains on a position goal",
       [](G &g) {
         g.gains.profile =
             rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_STIFF;
       }},
  };
  for (const auto &[what, mutate] : cases) {
    auto g = configs_goal(3);
    mutate(g);
    EXPECT_EQ(send<GoToJointConfigs>(rig.node, "go_to_joint_configs", g).code,
              kGoalRejected)
        << what;
  }
  EXPECT_TRUE(rig.fake.starts_seen().empty()) << "a refused goal was planned";
  EXPECT_EQ(rig.fake.blend_goals_received(), 0);
  EXPECT_FALSE(rig.sup.got_goal);
}

TEST_F(GotoManyTest, ConfigsLegsChainAndTheBlendExecutesAtSpeedScaleOne) {
  ConfigsRig rig("goto_configs_chain");
  auto g = configs_goal(3);
  g.speed_scale = {0.2, 0.5, 1.0};
  g.blend_radius = {0.02, 0.05};
  const auto r = send<GoToJointConfigs>(rig.node, "go_to_joint_configs", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  expect_chained(rig.fake, 3);
  const auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.speed_scale, std::vector<double>({0.2, 0.5, 1.0}));
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.02, 0.05}));
  EXPECT_TRUE(seen.hold.empty()) << "joint-space legs carry no hold";
  ASSERT_TRUE(rig.sup.got_goal);
  EXPECT_EQ(rig.sup.last_goal.trajectory.points.size(), 9u);
  EXPECT_DOUBLE_EQ(rig.sup.last_goal.speed_scale, 1.0);
}

TEST_F(GotoManyTest, ConfigsAFailingLegIsNamedAndNothingMoves) {
  ConfigsRig rig("goto_configs_leg_fails");
  rig.fake.set_fail_plan_number(1);
  const auto r =
      send<GoToJointConfigs>(rig.node, "go_to_joint_configs", configs_goal(2));
  EXPECT_EQ(r.code, result_code::kPlanningFailed);
  EXPECT_EQ(r.error, "leg 1/2: fake planner: no solution");
  EXPECT_EQ(rig.fake.starts_seen().size(), 1u) << "leg 2 must not be planned";
  EXPECT_EQ(rig.fake.blend_goals_received(), 0);
  EXPECT_FALSE(rig.sup.got_goal);
}
