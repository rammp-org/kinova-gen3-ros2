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
  // The measured configuration the arm is standing in. The planner must be
  // told to plan FROM this, rather than left to discover it for itself.
  kinova::JointVec q_meas = kinova::JointVec::Zero();
  ArmState on_query_state() override { ArmState s; s.q = q_meas; return s; }
};
// Cancels and joins the spin thread on ANY exit, so an early return from a
// failed ASSERT_* cannot destroy a joinable thread -- which calls
// std::terminate and replaces the gtest diagnostic with a bare SIGABRT.
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

// The planner must not have to discover where the arm is. kinova_arm_node owns
// the measured joint state, so it states it in the plan request; leaving
// start_joints empty is an instruction to the real cuRobo node to subscribe to
// /joint_states and work it out, which couples the planner to the robot and
// lets it plan from a state up to 2 s older than the one we execute from.
TEST_F(GotoServerTest, PlanRequestCarriesTheMeasuredStartConfiguration) {
  auto node = std::make_shared<rclcpp::Node>("goto_it_start");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  sup.q_meas << 0.11, -0.22, 0.33, -0.44, 0.55, -0.66, 0.77;
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);   // cancels + joins on any exit

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kSuccessful);

  const std::vector<double> seen = fake.last_start_joints();
  ASSERT_EQ(seen.size(), static_cast<size_t>(kinova::kNumJoints))
      << "planner was left to source the start state itself";
  for (int i = 0; i < kinova::kNumJoints; ++i)
    EXPECT_DOUBLE_EQ(seen[i], sup.q_meas[i]) << "joint_" << (i + 1);
}

// Finding #2 (fail-loud on planned-trajectory width): the fake planner reports
// SUCCESS but its first point carries only 4 positions instead of 7. on_plan_done
// must reject this before mapping/submitting rather than let to_trajectory_goal's
// zero-fill silently mis-map a short point onto the arm.
TEST_F(GotoServerTest, PlanSuccessWithMalformedWidthSettlesPlanningFailed) {
  auto node = std::make_shared<rclcpp::Node>("goto_it4");
  kinova_arm_ros2::test::FakeCuroboServer fake(
      node, /*succeed=*/true, /*n_points=*/3, /*reject=*/false, /*gate=*/{},
      /*started=*/nullptr, /*reject_cancel=*/false, /*bad_width=*/true);
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

// Finding #1 (cancel-then-execute): the goal is canceled WHILE still planning,
// but the (uninterruptible, per reject_cancel) fake planner still succeeds
// afterwards. on_plan_done must re-check is_canceling() on the plan-success
// path and settle PREEMPTED without ever submitting to the CommandSink.
//
// Deterministic sequencing, no sleeps: a gate blocks the fake planner's
// execute() until released; a "started" promise proves planning is in flight
// (and thus the goal is already recorded in GoToEEPoseServer's goals_ map)
// before the client cancels; async_cancel_goal's future only resolves after
// rclcpp_action has already flipped the outer goal's state to CANCELING on
// the server side, so releasing the gate afterwards deterministically races
// the plan success against an already-accepted cancel -- not a real race at
// the test level.
TEST_F(GotoServerTest, CancelDuringPlanningThenPlanSucceedsSettlesPreempted) {
  auto node = std::make_shared<rclcpp::Node>("goto_it3");
  auto started = std::make_shared<std::promise<void>>();
  auto started_future = started->get_future();
  std::promise<void> release;
  std::shared_future<void> gate = release.get_future().share();

  kinova_arm_ros2::test::FakeCuroboServer fake(
      node, /*succeed=*/true, /*n_points=*/3, /*reject=*/false, gate, started,
      /*reject_cancel=*/true);
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

  // FakeCuroboServer's action server sits on node's default (MutuallyExclusive)
  // callback group and is about to block inside execute(). Put the test's own
  // GoToEEPose client on a SEPARATE group so its cancel-response processing
  // isn't starved behind that blocked callback.
  auto client_grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose", client_grp);
  ASSERT_TRUE(client->wait_for_action_server(5s));
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = "base_link";

  std::promise<int> code_promise;
  auto code_future = code_promise.get_future();
  rclcpp_action::Client<GoToEEPose>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToEEPose>::WrappedResult& wr) {
        code_promise.set_value(wr.result ? wr.result->error_code : -12345);
      };
  auto goal_handle_future = client->async_send_goal(goal, opts);
  ASSERT_EQ(goal_handle_future.wait_for(5s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Block until the fake cuRobo server has actually entered execute() (i.e.
  // GoToEEPoseServer has already recorded the goal, still non-executing).
  ASSERT_EQ(started_future.wait_for(5s), std::future_status::ready);

  // Cancel the outer GoToEEPose goal; the future only resolves once the
  // server has already accepted the cancel (goal state -> CANCELING).
  auto cancel_future = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(5s), std::future_status::ready);

  // Now let the fake planner "finish" -- it still reports success even
  // though its own cancel was rejected, modeling a planner that committed
  // to an answer before honoring the cancel request.
  release.set_value();

  ASSERT_EQ(code_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(code_future.get(), result_code::kPreempted);
  EXPECT_FALSE(sup.got_goal);

  ex.cancel();
  spin.join();
}
