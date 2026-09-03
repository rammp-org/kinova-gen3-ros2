#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "kinova_lowlevel/interface/arbiter.h"
#include "kinova_lowlevel/interface/ports.h"
using namespace kinova::interface;
using namespace std::chrono_literals;

namespace {
// Records what actually reached the Supervisor's side of the Arbiter, and can
// be told to block inside a delegated call so the e-stop path can be raced
// against it.
struct RecordingSink : public CommandSink,
                       public StreamSink,
                       public GripperSink {
  mutable std::mutex m;
  std::vector<std::string> calls;
  std::atomic<bool> block_goal{false};
  std::atomic<bool> release_goal{false};
  std::atomic<bool> block_open{false};
  std::atomic<bool> release_open{false};
  Token last_setpoint_token{};

  void note(const std::string &s) {
    std::lock_guard<std::mutex> l(m);
    calls.push_back(s);
  }
  std::vector<std::string> log() const {
    std::lock_guard<std::mutex> l(m);
    return calls;
  }

  // CommandSink
  GoalResponse on_trajectory_goal(const TrajectoryGoal &) override {
    note("goal");
    while (block_goal.load() && !release_goal.load())
      std::this_thread::sleep_for(1ms);
    return GoalResponse::kAccept;
  }
  void on_trajectory_accepted(const GoalId &, const TrajectoryGoal &) override {
    note("accepted");
  }
  CancelResponse on_trajectory_cancel(const CancelRequest &) override {
    note("cancel");
    return CancelResponse::kAccept;
  }
  GainsResult on_set_gains(const GainsRequest &) override { return {}; }
  ArmState on_query_state() override {
    ArmState s;
    s.stamp_s = 1.0;
    return s;
  }
  void on_halt(HaltReason) override { note("halt"); }

  // StreamSink -- the Arbiter needs a downstream for it; unused by these tests.
  StreamOpenResult on_stream_open(const StreamOpenRequest &) override {
    note("stream_open");
    // Stands in for Supervisor's mode settle, which really does sleep 250 ms
    // inside this call while the Arbiter mutex is held.
    while (block_open.load() && !release_open.load())
      std::this_thread::sleep_for(1ms);
    return {};
  }
  void on_stream_close(const StreamCloseRequest &) override {}
  void on_setpoint_joint_position(const JointSetpoint &s) override {
    note("setpoint");
    std::lock_guard<std::mutex> l(m);
    last_setpoint_token = s.token;
  }
  void on_setpoint_joint_velocity(const JointSetpoint &) override {}
  void on_setpoint_joint_torque(const JointSetpoint &) override {}
  void on_setpoint_pose(const PoseSetpoint &) override {}
  void on_setpoint_twist(const TwistSetpoint &) override {}
  // Ungated in the Arbiter, so it must exist here even though these tests never
  // read it -- see core PR #31.
  StreamStatus on_query_stream() override {
    note("query_stream");
    return {};
  }

  // GripperSink -- the Arbiter gained a third downstream with core's gripper
  // tier
  // (#35). These tests do not exercise the gripper; the sink exists so the
  // Arbiter can construct, and records the call so a future test can assert the
  // gating.
  void on_gripper_setpoint(const GripperSetpoint &) override {
    note("gripper_setpoint");
  }
  GripperState on_query_gripper() override {
    note("query_gripper");
    return {};
  }
};

GoalId mkid(uint8_t x) {
  GoalId id{};
  id[0] = x;
  return id;
}
bool logged(const RecordingSink &s, const std::string &what) {
  for (const auto &c : s.log())
    if (c == what)
      return true;
  return false;
}
} // namespace

// The regression gate for the stored-token replay. A cancel carrying the goal's
// token must be admitted.
TEST(ArbitrationIntegration, CancelWithTheGoalsTokenIsAdmittedUnderEnforced) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  EXPECT_EQ(arb.on_trajectory_cancel({mkid(1), g.token}),
            CancelResponse::kAccept);
  EXPECT_TRUE(logged(sink, "cancel"));
}

// ...and the bug it exists to prevent: a zero token means the cancel never
// reaches the Supervisor, so the arm keeps moving while the client believes it
// cancelled.
TEST(ArbitrationIntegration, CancelWithAZeroTokenIsRefusedUnderEnforced) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  ASSERT_TRUE(arb.grant("orchestrator").accepted);

  EXPECT_EQ(arb.on_trajectory_cancel({mkid(1), Token{}}),
            CancelResponse::kReject);
  EXPECT_FALSE(logged(sink, "cancel"));
}

// kDisabled admits the zero token: the backward-compatibility gate that keeps
// every existing script working under the default mode.
TEST(ArbitrationIntegration, ZeroTokenIsAdmittedUnderDisabled) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kDisabled, 1234};
  TrajectoryGoal tg;
  EXPECT_EQ(arb.on_trajectory_goal(tg), GoalResponse::kAccept);
}

// Acquiring seizes: the incumbent's token stops working and generation bumps,
// which is how a dispossessed client detects what happened.
TEST(ArbitrationIntegration, StaleTokenIsRejectedAfterSeizure) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult first = arb.grant("teleop");
  ASSERT_TRUE(first.accepted);
  const GrantResult second = arb.grant("orchestrator");
  ASSERT_TRUE(second.accepted);
  EXPECT_EQ(second.generation, first.generation + 1);

  TrajectoryGoal stale;
  stale.token = first.token;
  EXPECT_EQ(arb.on_trajectory_goal(stale), GoalResponse::kRejectUnauthorized);
  TrajectoryGoal fresh;
  fresh.token = second.token;
  EXPECT_EQ(arb.on_trajectory_goal(fresh), GoalResponse::kAccept);
}

// THE safety test.
//
// NOTE ON WHAT IS BEING MEASURED: estop() *does* block on the arbiter mutex at
// its tail, by design -- the ownership bookkeeping and the second halt delivery
// both run under m_. Timing how long `arb.estop()` takes to RETURN would
// therefore just measure the blocked delegate and always fail. The property
// core actually engineered is that the LATCH and the FIRST HALT both happen
// BEFORE it ever contends for the lock. So: run estop() on its own thread and
// watch for the halt arriving downstream while the delegate is still stuck
// holding the mutex.
TEST(ArbitrationIntegration, EstopHaltReachesTheArmWhileADelegatedCallBlocks) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  sink.block_goal = true;
  TrajectoryGoal tg;
  tg.token = g.token;
  std::thread busy(
      [&] { arb.on_trajectory_goal(tg); }); // blocks HOLDING the mutex
  std::this_thread::sleep_for(100ms);

  std::thread stopper([&] { arb.estop(); });

  bool halted = false;
  const auto t0 = std::chrono::steady_clock::now();
  double waited = 0.0;
  while (waited < 1.0) {
    if (logged(sink, "halt")) {
      halted = true;
      break;
    }
    std::this_thread::sleep_for(5ms);
    waited =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
  }

  EXPECT_TRUE(halted)
      << "the halt never reached the arm while a delegate held the mutex";
  EXPECT_LT(waited, 0.5) << "halt queued behind the delegated call (" << waited
                         << "s)";

  sink.block_goal = false;
  sink.release_goal = true;
  busy.join();
  stopper.join();

  EXPECT_TRUE(arb.status().estopped);
  // And nothing is admitted afterwards -- the latch is the one thing kDisabled
  // does not bypass either.
  EXPECT_EQ(arb.on_trajectory_goal(tg), GoalResponse::kRejectUnauthorized);
}

// Setpoints are gated exactly like goals -- and the rejection is SILENT,
// because on_setpoint_* returns void. The Arbiter's rejected_count is the only
// signal there is.
TEST(ArbitrationIntegration, SetpointWithAStaleTokenIsDroppedSilently) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult first = arb.grant("teleop");
  ASSERT_TRUE(first.accepted);
  const GrantResult second =
      arb.grant("orchestrator"); // seizes; first token is dead
  ASSERT_TRUE(second.accepted);

  JointSetpoint stale;
  stale.token = first.token;
  arb.on_setpoint_joint_position(stale);
  EXPECT_FALSE(logged(sink, "setpoint")); // never reached the Supervisor
  const uint64_t rejected = arb.status().rejected_count;
  EXPECT_GT(rejected, 0u);

  JointSetpoint fresh;
  fresh.token = second.token;
  arb.on_setpoint_joint_position(fresh);
  EXPECT_TRUE(logged(sink, "setpoint"));
  EXPECT_EQ(sink.last_setpoint_token, second.token);
  EXPECT_EQ(arb.status().rejected_count,
            rejected); // an accepted one adds nothing
}

// A read must work with no owner and while e-stopped, or it is useless for
// working out why you were refused. Mirrors core's own
// Arbiter.QueryStreamIsNeverGated.
TEST(ArbitrationIntegration, QueryStreamIsNeverGated) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.on_query_stream();
  arb.estop();
  arb.on_query_stream();
  EXPECT_EQ(arb.status().rejected_count, 0u); // a read is not a rejection
}

// THE scenario core's lock-free estop() was actually written for, and which the
// arbitration tier could only approximate with a blocked goal: a real
// on_stream_open holding the Arbiter mutex through its mode settle while an
// e-stop arrives.
//
// As in the goal version, this measures the HALT ARRIVING downstream, not how
// long estop() takes to return -- estop() blocks on the mutex at its tail by
// design.
TEST(ArbitrationIntegration, EstopHaltReachesTheArmDuringAStreamOpen) {
  RecordingSink sink;
  Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  sink.block_open = true;
  StreamOpenRequest r;
  r.kind = SetpointKind::kJointPosition;
  r.control_mode = ControlModeKind::kPosition;
  r.timeout_s = 0.1;
  r.token = g.token;
  std::thread busy([&] { arb.on_stream_open(r); }); // blocks HOLDING the mutex
  std::this_thread::sleep_for(100ms);

  std::thread stopper([&] { arb.estop(); });
  bool halted = false;
  const auto t0 = std::chrono::steady_clock::now();
  double waited = 0.0;
  while (waited < 1.0) {
    if (logged(sink, "halt")) {
      halted = true;
      break;
    }
    std::this_thread::sleep_for(5ms);
    waited =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
  }
  EXPECT_TRUE(halted) << "the halt never reached the arm during a stream open";
  EXPECT_LT(waited, 0.5) << "halt queued behind the mode settle (" << waited
                         << "s)";

  sink.block_open = false;
  sink.release_open = true;
  busy.join();
  stopper.join();
  EXPECT_TRUE(arb.status().estopped);
}
