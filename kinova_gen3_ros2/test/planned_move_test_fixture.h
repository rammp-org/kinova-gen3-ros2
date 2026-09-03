#pragma once
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "kinova_lowlevel/interface/ports.h"

// Shared harness for the PlannedMoveServer-based integration tests
// (GoToJointConfig, GoToPreset). goto_ee_pose_integration_test deliberately
// keeps its own copies: it is the regression gate for the base-class
// extraction and has to stay unchanged to prove that refactor.

// Sentinels distinguishable from any real result_code (all >= -7).
inline constexpr int kServerMissing = 901;
inline constexpr int kTimedOut      = 902;
inline constexpr int kNoResult      = 903;
inline constexpr int kGoalRejected  = 904;   // the server refused the goal outright

// Stand-in for the Supervisor: records the submitted goal and, on accept,
// immediately drives the router as if execution completed successfully.
struct FakeSupervisor : public kinova::interface::CommandSink {
  kinova::interface::ActionServerPort& port;
  bool accept = true;
  bool got_goal = false;
  kinova::interface::TrajectoryGoal last_goal;
  explicit FakeSupervisor(kinova::interface::ActionServerPort& p) : port(p) {}
  kinova::interface::GoalResponse on_trajectory_goal(
      const kinova::interface::TrajectoryGoal& g) override {
    last_goal = g;
    got_goal = true;
    return accept ? kinova::interface::GoalResponse::kAccept
                  : kinova::interface::GoalResponse::kReject;
  }
  void on_trajectory_accepted(const kinova::interface::GoalId& id,
                              const kinova::interface::TrajectoryGoal&) override {
    kinova::interface::TrajectoryResult r;
    r.error_code = kinova::interface::result_code::kSuccessful;
    r.final_error = kinova::JointVec::Zero();
    port.settle(id, r);   // simulate immediate successful execution
  }
  kinova::interface::CancelResponse on_trajectory_cancel(
      const kinova::interface::CancelRequest& req) override {
    kinova::interface::TrajectoryResult r;
    r.error_code = kinova::interface::result_code::kPreempted;
    port.settle(req.id, r);
    return kinova::interface::CancelResponse::kAccept;
  }
  // Ownership revocation and /estop both land here. Nothing arbitrates in front
  // of this fake, so it only records that the halt arrived.
  bool halted = false;
  kinova::interface::HaltReason halt_reason{};
  void on_halt(kinova::interface::HaltReason why) override {
    halted = true; halt_reason = why;
  }
  kinova::interface::GainsResult on_set_gains(
      const kinova::interface::GainsRequest&) override { return {}; }
  // stamp_s > 0 marks the state as actually measured. The server refuses goals
  // when it is zero, because Supervisor::pump_loop only stores a snapshot after
  // a successful feedback read and {q=Zero, stamp_s=0} would otherwise be
  // executed as if the arm were really at zero.
  kinova::interface::ArmState on_query_state() override {
    kinova::interface::ArmState s; s.stamp_s = 1.0; return s;
  }
};

// Default router port; unused by these tests.
struct DummyPort : public kinova::interface::ActionServerPort {
  void publish_feedback(const kinova::interface::GoalId&,
                        const kinova::interface::TrajectoryFeedback&) override {}
  void settle(const kinova::interface::GoalId&,
              const kinova::interface::TrajectoryResult&) override {}
};

// Spins an executor on a background thread and always cancels + joins it, so an
// early return from a failed ASSERT_* cannot destroy a joinable thread (which
// calls std::terminate and replaces the gtest diagnostic with a bare SIGABRT).
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
