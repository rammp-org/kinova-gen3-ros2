#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <limits>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_interfaces/action/go_to_joint_config.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/goto_joint_config_server.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_curobo_server.h"
#include "planned_move_test_fixture.h"
using namespace std::chrono_literals;
using namespace kinova::interface;
using GoToJointConfig = kinova_arm_interfaces::action::GoToJointConfig;

namespace {
const std::array<double, 7> kTarget = {0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571};

// Send a goal and block for its result code; kGoalRejected if not accepted.
int send_and_get_code(rclcpp::Node::SharedPtr node, const std::array<double, 7>& joints) {
  auto client = rclcpp_action::create_client<GoToJointConfig>(node, "go_to_joint_config");
  if (!client->wait_for_action_server(5s)) return kServerMissing;
  GoToJointConfig::Goal goal;
  goal.target_joints = joints;
  std::promise<int> code;
  auto fut = code.get_future();
  rclcpp_action::Client<GoToJointConfig>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToJointConfig>::WrappedResult& wr) {
        code.set_value(wr.result ? wr.result->error_code : kNoResult);
      };
  auto gh_future = client->async_send_goal(goal, opts);
  if (gh_future.wait_for(5s) != std::future_status::ready) return kTimedOut;
  if (gh_future.get() == nullptr) return kGoalRejected;   // never accepted
  if (fut.wait_for(8s) != std::future_status::ready) return kTimedOut;
  return fut.get();
}
}  // namespace

class GotoJointConfigTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(GotoJointConfigTest, PlanSuccessDrivesTrajectoryAndSucceeds) {
  auto node = std::make_shared<rclcpp::Node>("goto_jc_it1");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToJointConfigServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  EXPECT_EQ(send_and_get_code(node, kTarget), result_code::kSuccessful);
  EXPECT_TRUE(sup.got_goal);
  EXPECT_EQ(sup.last_goal.trajectory.points.size(), 3u);
  EXPECT_EQ(sup.last_goal.control_mode, ControlModeKind::kPosition);
}

TEST_F(GotoJointConfigTest, PlanFailureSettlesPlanningFailed) {
  auto node = std::make_shared<rclcpp::Node>("goto_jc_it2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToJointConfigServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  EXPECT_EQ(send_and_get_code(node, kTarget), result_code::kPlanningFailed);
  EXPECT_FALSE(sup.got_goal) << "a failed plan must never reach the supervisor";
}

// float64[7] makes a wrong joint COUNT unrepresentable, so finiteness is the
// only reachable malformed goal. It must be rejected outright, not planned.
TEST_F(GotoJointConfigTest, NonFiniteTargetIsRejected) {
  auto node = std::make_shared<rclcpp::Node>("goto_jc_it3");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToJointConfigServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  auto bad = kTarget;
  bad[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(send_and_get_code(node, bad), kGoalRejected);
  EXPECT_FALSE(sup.got_goal);
}
