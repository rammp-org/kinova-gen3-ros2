#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <vector>
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
  // The measured configuration the arm is standing in. stamp_s must be > 0:
  // the supervisor only stores a snapshot after a SUCCESSFUL feedback read, so
  // a zero stamp means "no measurement yet", not "the arm is at zero".
  kinova::JointVec q_meas = kinova::JointVec::Zero();
  double stamp_s = 1.0;
  ArmState on_query_state() override {
    ArmState s; s.q = q_meas; s.stamp_s = stamp_s; return s;
  }
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

// A Supervisor stand-in that models the real ordering constraint the arm node
// lives under: a cancel only stops motion if it reaches the supervisor AFTER the
// goal has been handed over. The real Supervisor queues both on one FIFO inbox,
// so a cancel delivered first drains against no active goal and is silently
// lost -- and the trajectory then runs to completion.
//
// It blocks inside on_trajectory_goal until the test releases it, which pins the
// cancel to exactly the window between the supervisor accepting the goal and the
// server handing it over. Ordering is recorded, not timed.
struct OrderingSupervisor : public CommandSink {
  ActionServerPort& port;
  std::promise<void> in_goal;          // fires once on_trajectory_goal is entered
  std::shared_future<void> release;    // test releases it after the cancel lands
  std::mutex m;
  std::vector<std::string> calls;      // "goal" / "accepted" / "cancel"

  explicit OrderingSupervisor(ActionServerPort& p) : port(p) {}
  void note(const char* what) { std::lock_guard<std::mutex> l(m); calls.emplace_back(what); }
  std::vector<std::string> log() { std::lock_guard<std::mutex> l(m); return calls; }

  GoalResponse on_trajectory_goal(const TrajectoryGoal&) override {
    note("goal");
    in_goal.set_value();
    if (release.valid()) release.wait();
    return GoalResponse::kAccept;
  }
  void on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override {
    note("accepted");   // deliberately does NOT settle: the motion is "running"
  }
  CancelResponse on_trajectory_cancel(const GoalId& id) override {
    note("cancel");
    TrajectoryResult r; r.error_code = result_code::kPreempted;
    r.final_error = kinova::JointVec::Zero();
    port.settle(id, r);
    return CancelResponse::kAccept;
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

// A cancel accepted while the server is handing the planned trajectory to the
// supervisor must still stop the arm. Before the fix the cancel was ACCEPTed and
// then dropped: it either hit planner_.cancel() on an already-finished plan, or
// reached the supervisor AHEAD of the goal, where it drained against nothing.
// Either way the client saw a cancel it never got, and the arm executed the
// entire cuRobo motion at planned speed.
TEST_F(GotoServerTest, CancelDuringHandoverStillReachesTheSupervisor) {
  auto node = std::make_shared<rclcpp::Node>("goto_it_cancelwin");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  OrderingSupervisor sup(router);
  auto gate = std::make_shared<std::promise<void>>();
  sup.release = gate->get_future().share();
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = "base_link";
  auto gh_fut = client->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(5s), std::future_status::ready);
  auto gh = gh_fut.get();
  ASSERT_TRUE(gh);

  // Wait until the server is inside the handover, then cancel and release it.
  ASSERT_EQ(sup.in_goal.get_future().wait_for(5s), std::future_status::ready);
  auto cancel_fut = client->async_cancel_goal(gh);
  ASSERT_EQ(cancel_fut.wait_for(5s), std::future_status::ready);
  gate->set_value();

  // The supervisor must be told to cancel, and only AFTER it owns the goal.
  std::vector<std::string> seen;
  for (int i = 0; i < 100; ++i) {
    seen = sup.log();
    if (seen.size() >= 3) break;
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GE(seen.size(), 3u) << "supervisor never saw the cancel; the arm would "
                                "have run the whole trajectory";
  EXPECT_EQ(seen[0], "goal");
  EXPECT_EQ(seen[1], "accepted");
  EXPECT_EQ(seen[2], "cancel") << "cancel must arrive after the handover";
}

// The planner must not have to discover where the arm is: kinova_arm_node owns
// the measured joint state and states it in the plan request.
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
  SpinThread spin(ex);

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kSuccessful);

  const std::vector<double> seen = fake.last_start_joints();
  ASSERT_EQ(seen.size(), static_cast<size_t>(kinova::kNumJoints))
      << "planner was left to source the start state itself";
  for (int i = 0; i < kinova::kNumJoints; ++i)
    EXPECT_DOUBLE_EQ(seen[i], sup.q_meas[i]) << "joint_" << (i + 1);
}

// Supervisor::pump_loop stores a snapshot ONLY after a successful feedback read,
// and a default-constructed ArmState is {q = Zero, stamp_s = 0}. Before the first
// good frame, sending that q as the start state is indistinguishable from a real
// measurement, and cuRobo would plan from the fully-extended zero pose. The goal
// must be refused instead -- the loud failure this replaced lived on the planner
// side ("no fresh joint state on /joint_states").
TEST_F(GotoServerTest, GoalIsRefusedWhenNoJointStateHasBeenMeasuredYet) {
  auto node = std::make_shared<rclcpp::Node>("goto_it_nostate");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  sup.stamp_s = 0.0;                 // no successful feedback read has happened
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = "base_link";
  auto fut = client->async_send_goal(goal);
  ASSERT_EQ(fut.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(fut.get()) << "goal must be refused when the arm state is unknown";
  EXPECT_TRUE(fake.last_start_joints().empty())
      << "the planner must not have been asked to plan from a fabricated pose";
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
