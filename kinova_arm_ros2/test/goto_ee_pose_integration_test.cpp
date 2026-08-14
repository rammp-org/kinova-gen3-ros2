#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/goto_ee_pose_server.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_curobo_server.h"
using namespace std::chrono_literals;
using namespace kinova::interface;
using GoToEEPose = kinova_arm_interfaces::action::GoToEEPose;

namespace {
// Stand-in for the Supervisor: records the submitted goal and, on accept,
// immediately drives the router as if execution completed successfully.
struct FakeSupervisor : public CommandSink {
  ActionServerPort& port;
  bool accept = true;
  bool got_goal = false;
  TrajectoryGoal last_goal;
  explicit FakeSupervisor(ActionServerPort& p) : port(p) {}
  GoalResponse on_trajectory_goal(const TrajectoryGoal& g) override {
    last_goal = g; got_goal = true;
    return accept ? GoalResponse::kAccept : GoalResponse::kReject;
  }
  void on_trajectory_accepted(const GoalId& id, const TrajectoryGoal&) override {
    TrajectoryResult r; r.error_code = result_code::kSuccessful;
    r.final_error = kinova::JointVec::Zero();
    port.settle(id, r);   // simulate immediate successful execution
  }
  CancelResponse on_trajectory_cancel(const GoalId& id) override {
    TrajectoryResult r; r.error_code = result_code::kPreempted;
    port.settle(id, r); return CancelResponse::kAccept;
  }
  GainsResult on_set_gains(const GainsRequest&) override { return {}; }
  ArmState on_query_state() override { return {}; }
};
struct DummyPort : public ActionServerPort {   // default router port; unused here
  void publish_feedback(const GoalId&, const TrajectoryFeedback&) override {}
  void settle(const GoalId&, const TrajectoryResult&) override {}
};

// Send a GoToEEPose goal and block for its result code.
int send_and_get_code(rclcpp::Node::SharedPtr node, const std::string& frame) {
  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose");
  if (!client->wait_for_action_server(5s)) return 999;
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = frame;
  std::promise<int> code;
  auto fut = code.get_future();
  rclcpp_action::Client<GoToEEPose>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToEEPose>::WrappedResult& wr) {
        code.set_value(wr.result ? wr.result->error_code : -12345);
      };
  client->async_send_goal(goal, opts);
  if (fut.wait_for(8s) != std::future_status::ready) return 888;
  return fut.get();
}
}  // namespace

class GotoServerTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(GotoServerTest, PlanSuccessDrivesTrajectoryAndSucceeds) {
  auto node = std::make_shared<rclcpp::Node>("goto_it");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kSuccessful);
  EXPECT_TRUE(sup.got_goal);
  EXPECT_EQ(sup.last_goal.trajectory.points.size(), 3u);
  EXPECT_EQ(sup.last_goal.control_mode, ControlModeKind::kPosition);

  ex.cancel();
  spin.join();
}

TEST_F(GotoServerTest, PlanFailureSettlesPlanningFailed) {
  auto node = std::make_shared<rclcpp::Node>("goto_it2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kPlanningFailed);
  EXPECT_FALSE(sup.got_goal);

  ex.cancel();
  spin.join();
}
