#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <limits>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_arm_interfaces/action/go_to_preset.hpp"
#include "kinova_gen3_ros2/curobo_plan_client.h"
#include "kinova_gen3_ros2/goal_router.h"
#include "kinova_gen3_ros2/goto_preset_server.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_curobo_server.h"
#include "planned_move_test_fixture.h"
using namespace std::chrono_literals;
using namespace kinova::interface;
using GoToPreset = rammp_arm_interfaces::action::GoToPreset;

namespace {
kinova_gen3_ros2::GoToPresetServer::Registry test_registry() {
  return {
      {"home",
       {0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571}}}; // Kinova factory Home
}

int send_and_get_code(rclcpp::Node::SharedPtr node, const std::string &preset,
                      double speed_scale = 1.0) {
  auto client = rclcpp_action::create_client<GoToPreset>(node, "go_to_preset");
  if (!client->wait_for_action_server(5s))
    return kServerMissing;
  GoToPreset::Goal goal;
  goal.preset_name = preset;
  goal.speed_scale = speed_scale;
  std::promise<int> code;
  auto fut = code.get_future();
  rclcpp_action::Client<GoToPreset>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToPreset>::WrappedResult
              &wr) {
        code.set_value(wr.result ? wr.result->error_code : kNoResult);
      };
  auto gh_future = client->async_send_goal(goal, opts);
  if (gh_future.wait_for(5s) != std::future_status::ready)
    return kTimedOut;
  if (gh_future.get() == nullptr)
    return kGoalRejected;
  if (fut.wait_for(8s) != std::future_status::ready)
    return kTimedOut;
  return fut.get();
}
} // namespace

class GotoPresetTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(GotoPresetTest, KnownPresetPlansAndSucceeds) {
  auto node = std::make_shared<rclcpp::Node>("goto_preset_it1");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router(dummy);
  kinova_gen3_ros2::GoToPresetServer server(node, router, planner, grp,
                                            test_registry());
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  EXPECT_EQ(send_and_get_code(node, "home"), result_code::kSuccessful);
  EXPECT_TRUE(sup.got_goal);
  EXPECT_EQ(sup.last_goal.trajectory.points.size(), 3u);
  EXPECT_EQ(sup.last_goal.control_mode, ControlModeKind::kPosition);
}

// An unknown name must be refused outright. Silently substituting a default
// would move the arm somewhere the caller never asked for.
TEST_F(GotoPresetTest, UnknownPresetIsRejected) {
  auto node = std::make_shared<rclcpp::Node>("goto_preset_it2");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router(dummy);
  kinova_gen3_ros2::GoToPresetServer server(node, router, planner, grp,
                                            test_registry());
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  EXPECT_EQ(send_and_get_code(node, "nope"), kGoalRejected);
  EXPECT_FALSE(sup.got_goal);
}

TEST_F(GotoPresetTest, SpeedScaleReachesTheTrajectoryGoal) {
  auto node = std::make_shared<rclcpp::Node>("goto_preset_speed");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router(dummy);
  kinova_gen3_ros2::GoToPresetServer server(node, router, planner, grp,
                                            test_registry());
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  EXPECT_EQ(send_and_get_code(node, "home", 0.5), result_code::kSuccessful);
  EXPECT_DOUBLE_EQ(sup.last_goal.speed_scale, 0.5);
}

TEST_F(GotoPresetTest, AnUnusableSpeedScaleIsRefused) {
  auto node = std::make_shared<rclcpp::Node>("goto_preset_speed_bad");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router(dummy);
  kinova_gen3_ros2::GoToPresetServer server(node, router, planner, grp,
                                            test_registry());
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  for (double bad : {0.0, 1.5, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_EQ(send_and_get_code(node, "home", bad), kGoalRejected) << bad;
  EXPECT_FALSE(sup.got_goal);
}

// `home` and `rest` are the names callers code against: a stock node has both,
// at Kinova's factory Home and at Retract with joint 4 opened to -135 deg.
TEST_F(GotoPresetTest, StockRegistryHasHomeAndRest) {
  auto node = std::make_shared<rclcpp::Node>("goto_preset_reg_stock");
  const auto reg = kinova_gen3_ros2::load_preset_registry(*node);
  ASSERT_EQ(reg.size(), 2u);
  EXPECT_EQ(reg.at("home"),
            (std::vector<double>{0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571}));
  EXPECT_EQ(reg.at("rest"), (std::vector<double>{0.0, -0.349, 3.142, -2.356,
                                                 0.0, -0.873, 1.571}));
}

// A deployment that lists other presets still gets the required ones, and may
// override their angles.
TEST_F(GotoPresetTest, RequiredPresetsSurviveACustomListAndCanBeOverridden) {
  const std::vector<double> mine = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
      {rclcpp::Parameter("preset_names", std::vector<std::string>{"pour"}),
       rclcpp::Parameter("presets.pour", mine),
       rclcpp::Parameter("presets.rest", mine)});
  auto node = std::make_shared<rclcpp::Node>("goto_preset_reg_custom", opts);
  const auto reg = kinova_gen3_ros2::load_preset_registry(*node);
  ASSERT_EQ(reg.size(), 3u);
  EXPECT_EQ(reg.at("pour"), mine);
  EXPECT_EQ(reg.at("rest"), mine);
  EXPECT_EQ(reg.at("home"),
            (std::vector<double>{0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571}));
}
