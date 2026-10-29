#include <gtest/gtest.h>
#include <array>
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
using EEPoseTarget = rammp_arm_interfaces::msg::EEPoseTarget;
using JointConfigTarget = rammp_arm_interfaces::msg::JointConfigTarget;
using PlannerHold = rammp_curobo_interfaces::action::PlanToPose::Goal;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

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
// thread stops first. The fake's gate/started knobs are for the cancel test.
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
  explicit Rig(const char *name, std::shared_future<void> gate = {},
               std::shared_ptr<std::promise<void>> started = nullptr)
      : node(std::make_shared<rclcpp::Node>(name)),
        fake(node, /*succeed=*/true, /*n_points=*/3, /*reject=*/false,
             std::move(gate), std::move(started), /*reject_cancel=*/true),
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

// n targets with only the pose set: every other field at its default.
GoToEEPoses::Goal poses_goal(size_t n) {
  GoToEEPoses::Goal g;
  for (size_t k = 0; k < n; ++k) {
    EEPoseTarget t;
    t.pose.header.frame_id = "arm_mount";
    t.pose.pose.position.x = 0.4;
    t.pose.pose.position.y = 0.1 * static_cast<double>(k);
    t.pose.pose.position.z = 0.3;
    t.pose.pose.orientation.w = 1.0;
    g.targets.push_back(t);
  }
  return g;
}

// n targets with only the joints set.
GoToJointConfigs::Goal configs_goal(size_t n) {
  GoToJointConfigs::Goal g;
  for (size_t k = 0; k < n; ++k) {
    JointConfigTarget t;
    t.target_joints = {
        0.1 * static_cast<double>(k), 0.262, 3.142, -2.269, 0.0, 0.96, 1.571};
    g.targets.push_back(t);
  }
  return g;
}

// Each leg after the first must be planned from the last point of the leg the
// planner returned before it, and the first from the measured state (zero in
// FakeSupervisor). The fake blend also refuses legs that do not join.
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
       [](G &g) { g.targets[1].pose.header.frame_id = "base_link"; }},
      {"empty frame", [](G &g) { g.targets[2].pose.header.frame_id = ""; }},
      {"zero speed", [](G &g) { g.targets[0].speed_scale = 0.0; }},
      {"fast leg", [](G &g) { g.targets[1].speed_scale = 1.5; }},
      {"NaN speed", [](G &g) { g.targets[2].speed_scale = kNaN; }},
      {"unknown hold", [](G &g) { g.targets[1].orientation_hold = 3; }},
      {"negative radius", [](G &g) { g.targets[0].blend_radius = -0.01; }},
      {"NaN radius", [](G &g) { g.targets[1].blend_radius = kNaN; }},
      {"infinite radius", [](G &g) { g.targets[0].blend_radius = kInf; }},
      // Not used, but still checked: a bad value is a caller's bug.
      {"NaN radius on the last target",
       [](G &g) { g.targets[2].blend_radius = kNaN; }},
      {"negative radius on the last target",
       [](G &g) { g.targets[2].blend_radius = -0.01; }},
      {"NaN position", [](G &g) { g.targets[1].pose.pose.position.y = kNaN; }},
      {"infinite position",
       [](G &g) { g.targets[0].pose.pose.position.z = kInf; }},
      {"NaN orientation",
       [](G &g) { g.targets[2].pose.pose.orientation.x = kNaN; }},
      {"zero quaternion",
       [](G &g) { g.targets[1].pose.pose.orientation.w = 0; }},
      {"non-unit quaternion",
       [](G &g) { g.targets[0].pose.pose.orientation.w = 1.01; }},
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

// Leg k is planned TO targets[k], in order, from where leg k-1 ends; what
// executes is the blend of all of them.
TEST_F(GotoManyTest, PosesLegsArePlannedToEachTargetInOrderChained) {
  PosesRig rig("goto_poses_chain");
  const auto goal = poses_goal(3);
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", goal);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  const auto targets = rig.fake.pose_targets_seen();
  ASSERT_EQ(targets.size(), 3u);
  for (size_t k = 0; k < 3; ++k)
    EXPECT_EQ(targets[k], goal.targets[k].pose.pose) << "leg " << k + 1;
  expect_chained(rig.fake, 3);
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

// Each target's own fields reach the plan of its leg and the blend; the
// blend carries the speeds, so it executes at 1.0 (scaling it again would
// apply them twice).
TEST_F(GotoManyTest, PosesEachTargetsFieldsReachItsLegAndTheBlend) {
  PosesRig rig("goto_poses_fields");
  auto g = poses_goal(3);
  g.targets[0].speed_scale = 0.4;
  g.targets[0].orientation_hold = EEPoseTarget::HOLD_LEVEL;
  g.targets[0].blend_radius = 0.03;
  g.targets[1].speed_scale = 0.7;
  g.targets[1].orientation_hold = EEPoseTarget::HOLD_NONE;
  g.targets[2].orientation_hold = EEPoseTarget::HOLD_FIXED;
  g.targets[2].blend_radius = 0.5; // the last target's: not used
  g.targets[2].pose.pose.orientation.w = 0.0;
  g.targets[2].pose.pose.orientation.x = 1.0;
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;

  const std::vector<uint8_t> holds = {
      PlannerHold::HOLD_LEVEL, PlannerHold::HOLD_NONE, PlannerHold::HOLD_FIXED};
  EXPECT_EQ(rig.fake.holds_seen(), holds) << "each leg planned with its hold";
  const auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.hold, holds);
  EXPECT_EQ(seen.speed_scale, std::vector<double>({0.4, 0.7, 1.0}));
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.03, 0.0}));
  ASSERT_EQ(seen.hold_orientation.size(), 3u);
  for (size_t k = 0; k < 3; ++k)
    EXPECT_EQ(seen.hold_orientation[k], g.targets[k].pose.pose.orientation)
        << "leg " << k + 1;
  EXPECT_DOUBLE_EQ(rig.sup.last_goal.speed_scale, 1.0);
}

// A target with only its pose set: full speed, no hold, stop on it.
TEST_F(GotoManyTest, PosesTargetDefaults) {
  PosesRig rig("goto_poses_defaults");
  const auto r = send<GoToEEPoses>(rig.node, "go_to_ee_poses", poses_goal(3));
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  EXPECT_EQ(rig.fake.holds_seen(),
            std::vector<uint8_t>(3, PlannerHold::HOLD_NONE));
  const auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.speed_scale, std::vector<double>({1.0, 1.0, 1.0}));
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.0, 0.0}));
  EXPECT_EQ(seen.hold, std::vector<uint8_t>(3, PlannerHold::HOLD_NONE));
  EXPECT_DOUBLE_EQ(rig.sup.last_goal.speed_scale, 1.0);
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

// A cancel that lands while leg 2 of 3 is planning. The fake will not
// interrupt that plan (reject_cancel), so leg 2 still succeeds -- and the
// chain must then stop rather than plan leg 3 or the blend, because
// planner_.cancel() had nothing else in flight to reach.
TEST_F(GotoManyTest, PosesACancelDuringALegStopsTheChain) {
  // Declared before the rig, so they outlive its executor: a late result
  // must never land in a destroyed promise.
  std::promise<void> gate;
  auto started = std::make_shared<std::promise<void>>();
  auto leg2_started = started->get_future();
  std::promise<int> code;
  auto code_f = code.get_future();
  PosesRig rig("goto_poses_cancel", gate.get_future().share(), started);
  rig.fake.set_gated_plan_number(2);
  // Opens the gate on every way out, before the rig joins its executor --
  // otherwise an early ASSERT leaves a thread parked on it and the join hangs.
  struct OpenGate {
    std::promise<void> &p;
    bool open = false;
    void operator()() {
      if (!open)
        p.set_value();
      open = true;
    }
    ~OpenGate() { (*this)(); }
  } open_gate{gate};

  auto client =
      rclcpp_action::create_client<GoToEEPoses>(rig.node, "go_to_ee_poses");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  rclcpp_action::Client<GoToEEPoses>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToEEPoses>::WrappedResult
              &wr) { code.set_value(wr.result ? wr.result->error_code : 0); };
  auto gh_f = client->async_send_goal(poses_goal(3), opts);
  ASSERT_EQ(gh_f.wait_for(5s), std::future_status::ready);
  auto gh = gh_f.get();
  ASSERT_TRUE(gh);
  ASSERT_EQ(leg2_started.wait_for(5s), std::future_status::ready);

  auto cancel_f = client->async_cancel_goal(gh);
  ASSERT_EQ(cancel_f.wait_for(5s), std::future_status::ready);
  open_gate(); // only now let leg 2 finish
  ASSERT_EQ(code_f.wait_for(8s), std::future_status::ready);
  EXPECT_EQ(code_f.get(), result_code::kPreempted);
  EXPECT_EQ(rig.fake.starts_seen().size(), 2u) << "leg 3 must not be planned";
  EXPECT_EQ(rig.fake.blend_goals_received(), 0);
  EXPECT_FALSE(rig.sup.got_goal) << "a canceled goal must never reach the arm";
}

// --- GoToJointConfigs -------------------------------------------------------

TEST_F(GotoManyTest, ConfigsRefusesMalformedGoalsAtSubmission) {
  ConfigsRig rig("goto_configs_refuse");
  using G = GoToJointConfigs::Goal;
  const std::vector<std::pair<const char *, std::function<void(G &)>>> cases = {
      {"no targets", [](G &g) { g.targets.clear(); }},
      {"NaN joint", [](G &g) { g.targets[2].target_joints[3] = kNaN; }},
      {"infinite joint", [](G &g) { g.targets[0].target_joints[6] = kInf; }},
      {"zero speed", [](G &g) { g.targets[1].speed_scale = 0.0; }},
      {"fast leg", [](G &g) { g.targets[0].speed_scale = 1.5; }},
      {"negative radius", [](G &g) { g.targets[1].blend_radius = -1.0; }},
      {"NaN radius on the last target",
       [](G &g) { g.targets[2].blend_radius = kNaN; }},
      {"negative radius on the last target",
       [](G &g) { g.targets[2].blend_radius = -0.01; }},
      {"gains on a position goal",
       [](G &g) {
         g.gains.profile =
             rammp_arm_interfaces::msg::ImpedanceGains::PROFILE_STIFF;
       }},
      {"unknown control_mode", [](G &g) { g.control_mode = 5; }},
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

TEST_F(GotoManyTest, ConfigsLegsArePlannedToEachTargetInOrderChained) {
  ConfigsRig rig("goto_configs_chain");
  auto g = configs_goal(3);
  g.targets[0].speed_scale = 0.2;
  g.targets[0].blend_radius = 0.02;
  g.targets[1].speed_scale = 0.5;
  g.targets[1].blend_radius = 0.05;
  const auto r = send<GoToJointConfigs>(rig.node, "go_to_joint_configs", g);
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  const auto targets = rig.fake.joint_targets_seen();
  ASSERT_EQ(targets.size(), 3u);
  for (size_t k = 0; k < 3; ++k)
    EXPECT_EQ(targets[k],
              std::vector<double>(g.targets[k].target_joints.begin(),
                                  g.targets[k].target_joints.end()))
        << "leg " << k + 1;
  expect_chained(rig.fake, 3);
  const auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.speed_scale, std::vector<double>({0.2, 0.5, 1.0}));
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.02, 0.05}));
  EXPECT_TRUE(seen.hold.empty()) << "joint-space legs carry no hold";
  EXPECT_TRUE(seen.hold_orientation.empty());
  ASSERT_TRUE(rig.sup.got_goal);
  EXPECT_EQ(rig.sup.last_goal.trajectory.points.size(), 9u);
  EXPECT_DOUBLE_EQ(rig.sup.last_goal.speed_scale, 1.0);
}

// A target with only its joints set: full speed, stop on it.
TEST_F(GotoManyTest, ConfigsTargetDefaults) {
  ConfigsRig rig("goto_configs_defaults");
  const auto r =
      send<GoToJointConfigs>(rig.node, "go_to_joint_configs", configs_goal(2));
  ASSERT_EQ(r.code, result_code::kSuccessful) << r.error;
  const auto seen = rig.fake.last_blend();
  EXPECT_EQ(seen.speed_scale, std::vector<double>({1.0, 1.0}));
  EXPECT_EQ(seen.blend_radius, std::vector<double>({0.0}));
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
