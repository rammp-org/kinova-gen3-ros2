#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_curobo_interfaces/action/blend.hpp"
#include "rammp_curobo_interfaces/action/plan_to_joints.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_gen3_ros2::test {

// The model's arm joints in chain order, as bringup hands them to the GoTo*
// servers. The real planner emits these names (since RAMMP-CuRobo#38), and so
// does this fake unless told otherwise.
inline const std::vector<std::string> kArmJointNames = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7"};

// Minimal fake cuRobo planner. Hosts BOTH /rammp_curobo/plan_to_pose and
// /rammp_curobo/plan_to_joints off the same configuration, so a test picks a
// planning tier purely by which one it calls. succeed=true returns a canned
// n-point arm_joint_1..7 trajectory; succeed=false aborts with a message.
// reject=true rejects the goal outright (goal_response_callback sees a null
// handle).
//
// Optional, additive knobs for deterministically testing races against a
// caller that cancels while planning is in flight:
//   - gate: if valid, execute() blocks on it before producing a result, so a
//     test can pin down exactly when planning "finishes".
//   - started: if non-null, set once execute() begins (before waiting on
//     gate) so a test can deterministically know planning has started.
//     set_gated_plan_number(n) narrows both to the n-th plan, so a test can
//     hold one leg of a chain while the others run through.
//   - reject_cancel: if true, this server's OWN cancel handler REJECTs
//     incoming cancel requests -- models a planner that can't be interrupted
//     once committed, so execute() can still legitimately succeed() even
//     after a cancel was requested against its goal (without racing an
//     invalid CANCELING->SUCCEEDED transition in rclcpp_action).
//   - bad_width: if true, the first point of an otherwise-successful plan
//     carries only 4 positions instead of 7 -- models a malformed/buggy
//     planner response for testing the caller's fail-loud width guard.
//
// A plan starts EXACTLY at the start it was given and steps up from it, with
// velocities, at rest at both ends -- as the real planner's legs do -- so each
// leg ends somewhere different and a chain of legs can be followed. Every
// plan's start, target and hold is recorded in order. Every plan and blend
// publishes one feedback, state kPlanState / kBlendState, before its result.
// set_empty_plan_number(n) makes the n-th plan succeed with no points.
//
// Also hosts /rammp_curobo/blend. It refuses SOME of what the real one refuses:
// a leg with fewer than 2 points or a point without velocities, a leg not
// starting within 1e-5 rad of where the one before it ends, and per-leg or
// per-junction arrays of the wrong length. It does NOT model "planner busy",
// speed scales out of range, a zero-length hold orientation, or unevenly
// spaced points. It records the goal and returns the legs' points end to end
// (set_blend_succeed(false) aborts it instead).
class FakeCuroboServer {
public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using PlanToJoints = rammp_curobo_interfaces::action::PlanToJoints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanToPose>;
  using JointsGoalHandle = rclcpp_action::ServerGoalHandle<PlanToJoints>;
  using Blend = rammp_curobo_interfaces::action::Blend;
  using BlendGoalHandle = rclcpp_action::ServerGoalHandle<Blend>;

  // The frame the fake says it planned in. Set before any goal is sent.
  void set_frame_id(std::string f) { frame_id_ = std::move(f); }
  // The joint_names every plan carries. Set before any goal is sent.
  void set_joint_names(std::vector<std::string> n) {
    joint_names_ = std::move(n);
  }
  // The n-th plan goal executed (1-based, either tier) aborts. Set before any
  // goal is sent.
  void set_fail_plan_number(int n) { fail_plan_number_ = n; }
  void set_blend_succeed(bool b) { blend_succeed_ = b; }
  // Only the n-th plan (1-based, either tier) signals `started` and waits on
  // `gate`. 0, the default: every plan does. Set before any goal is sent.
  void set_gated_plan_number(int n) { gated_plan_number_ = n; }
  // The n-th plan (1-based, either tier) succeeds with an EMPTY trajectory.
  void set_empty_plan_number(int n) { empty_plan_number_ = n; }
  // The feedback state every plan / blend publishes.
  static constexpr const char *kPlanState = "fake planning";
  static constexpr const char *kBlendState = "fake blending";

  FakeCuroboServer(rclcpp::Node::SharedPtr node, bool succeed, int n_points = 3,
                   bool reject = false, std::shared_future<void> gate = {},
                   std::shared_ptr<std::promise<void>> started = nullptr,
                   bool reject_cancel = false, bool bad_width = false)
      : node_(node), succeed_(succeed), n_points_(n_points), reject_(reject),
        gate_(std::move(gate)), started_(std::move(started)),
        reject_cancel_(reject_cancel), bad_width_(bad_width) {
    // Its own group, so a plan held on `gate` blocks only this fake, not the
    // node's default group that a test's own clients answer on. Kept as a
    // member: the node holds its groups only weakly, and a dropped group's
    // callbacks never run.
    //
    // MUTUALLY EXCLUSIVE, not reentrant. A plan executes inside the server's
    // accept callback; on a reentrant group the caller's result request can be
    // handled on another thread meanwhile, and a succeed() that lands while
    // that request is being filed is lost under Humble's rclcpp_action -- with
    // feedback's 20 ms pause in the window, every multi-leg chain hung. Here
    // the request waits until the plan is done and is answered at once.
    group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto opts = rcl_action_server_get_default_options();
    server_ = rclcpp_action::create_server<PlanToPose>(
        node_, "/rammp_curobo/plan_to_pose",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const PlanToPose::Goal>) {
          ++pose_goals_received_;
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandle>) {
          return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT
                                : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> gh) { execute<PlanToPose>(gh); },
        opts, group_);
    joints_server_ = rclcpp_action::create_server<PlanToJoints>(
        node_, "/rammp_curobo/plan_to_joints",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const PlanToJoints::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<JointsGoalHandle>) {
          return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT
                                : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<JointsGoalHandle> gh) {
          execute<PlanToJoints>(gh);
        },
        opts, group_);
    blend_server_ = rclcpp_action::create_server<Blend>(
        node_, "/rammp_curobo/blend",
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const Blend::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<BlendGoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<BlendGoalHandle> gh) { execute_blend(gh); },
        opts, group_);
  }

  // What the caller actually asked us to plan FROM. Empty means the caller
  // left start_joints unset, which tells the real cuRobo node to go read
  // /joint_states itself -- the coupling this records the absence of.
  std::vector<double> last_start_joints() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return last_start_joints_;
  }

  // What the caller asked us to constrain the plan by. kNoHoldSeen until a
  // pose goal arrives -- deliberately NOT HOLD_NONE, so a test expecting
  // HOLD_NONE has to observe it rather than inherit it from a fake that was
  // never reached.
  static constexpr uint8_t kNoHoldSeen = 255;
  uint8_t last_hold() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return last_hold_;
  }
  // Still recorded, though no test asserts it: approach_via remains a field
  // on PlanToPose (shelved, see kinova-gen3-ros2#40) and this double should
  // mirror the real message rather than a subset of it.
  rammp_curobo_interfaces::msg::ApproachVia last_approach_via() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return last_approach_via_;
  }

  // Pose goals that reached this server at all, accepted or not. Lets a test
  // prove a plan() call was refused client-side rather than dispatched.
  int pose_goals_received() const { return pose_goals_received_; }

  // Every plan's start_joints, in the order the plans were executed.
  std::vector<std::vector<double>> starts_seen() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return starts_seen_;
  }
  // The last blend goal, and how many arrived.
  Blend::Goal last_blend() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return last_blend_;
  }
  int blend_goals_received() const { return blend_goals_received_; }
  // Every pose plan's target and hold, and every joint plan's target, in the
  // order the plans were executed.
  std::vector<geometry_msgs::msg::Pose> pose_targets_seen() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return pose_targets_seen_;
  }
  std::vector<uint8_t> holds_seen() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return holds_seen_;
  }
  std::vector<std::vector<double>> joint_targets_seen() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return joint_targets_seen_;
  }

private:
  void record_constraints(const PlanToPose::Goal &g) {
    last_hold_ = g.hold;
    last_approach_via_ = g.approach_via;
    pose_targets_seen_.push_back(g.target);
    holds_seen_.push_back(g.hold);
  }
  void record_constraints(const PlanToJoints::Goal &g) {
    joint_targets_seen_.push_back(g.target_joints);
  }
  // Shared by both tiers; only the Result type differs. PlanToJoints::Result
  // additionally carries goal_mismatch_rad, which stays at its 0.0 default -
  // the canned plan is treated as reaching the requested joints exactly.
  // `started_` is a one-shot promise, so without set_gated_plan_number a
  // single test must send only one plan.
  template <typename ActionT>
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh) {
    const auto start = gh->get_goal()->start_joints;
    {
      std::lock_guard<std::mutex> l(seen_m_);
      last_start_joints_ = start;
      starts_seen_.push_back(start);
      record_constraints(*gh->get_goal());
    }
    const int nth = ++plans_executed_;
    const bool gated = gated_plan_number_ == 0 || nth == gated_plan_number_;
    if (started_ && gated)
      started_->set_value();
    if (gate_.valid() && gated)
      gate_.wait();
    publish_state(gh, kPlanState);
    auto result = std::make_shared<typename ActionT::Result>();
    if (!succeed_ || nth == fail_plan_number_) {
      result->success = false;
      result->message = "fake planner: no solution";
      gh->abort(result);
      return;
    }
    result->success = true;
    result->message = "fake plan ok";
    result->trajectory.header.frame_id = frame_id_;
    result->trajectory.joint_names = joint_names_;
    for (int k = 0; k < (nth == empty_plan_number_ ? 0 : n_points_); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign((bad_width_ && k == 0) ? 4 : 7, 0.01 * k);
      for (size_t i = 0; i < p.positions.size() && i < start.size(); ++i)
        p.positions[i] += start[i];
      p.velocities.assign(7, (k == 0 || k + 1 == n_points_) ? 0.0 : 0.5);
      const double t = 0.02 * (k + 1);
      p.time_from_start.sec = static_cast<int32_t>(t);
      p.time_from_start.nanosec =
          static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
      result->trajectory.points.push_back(p);
    }
    gh->succeed(result);
  }
  // One feedback, with a pause on each side. Feedback, goal response and
  // result travel separately, and a caller drops feedback for a goal it does
  // not know yet (its goal response not handled) or has already settled.
  template <typename ActionT>
  static void publish_state(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> &gh,
      const char *state) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto fb = std::make_shared<typename ActionT::Feedback>();
    fb->state = state;
    gh->publish_feedback(fb);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // Why this fake's blend refuses this goal, or "" if it does not: a subset of
  // the real blend's refusals (see the class comment).
  static std::string blend_refusal(const Blend::Goal &g) {
    const size_t n = g.legs.size();
    if (n == 0)
      return "no legs";
    if (g.blend_radius.size() != n - 1)
      return std::to_string(n) + " legs need " + std::to_string(n - 1) +
             " blend radii, got " + std::to_string(g.blend_radius.size());
    for (const size_t m :
         {g.speed_scale.size(), g.hold.size(), g.hold_orientation.size()})
      if (m != 0 && m != n)
        return "a per-leg array has " + std::to_string(m) + " values for " +
               std::to_string(n) + " legs";
    for (size_t k = 0; k < n; ++k) {
      const auto &pts = g.legs[k].points;
      const std::string what =
          "leg " + std::to_string(k + 1) + "/" + std::to_string(n) + ": ";
      if (pts.size() < 2)
        return what + "fewer than 2 points";
      for (const auto &p : pts)
        if (p.velocities.size() != p.positions.size())
          return what + "a point without velocities";
      if (k == 0)
        continue;
      const auto &a = g.legs[k - 1].points.back().positions;
      const auto &b = pts.front().positions;
      if (a.size() != b.size())
        return what + "does not start where leg " + std::to_string(k) + " ends";
      for (size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-5)
          return what + "starts more than 1e-5 rad from where leg " +
                 std::to_string(k) + " ends";
    }
    return "";
  }
  void execute_blend(std::shared_ptr<BlendGoalHandle> gh) {
    {
      std::lock_guard<std::mutex> l(seen_m_);
      last_blend_ = *gh->get_goal();
    }
    ++blend_goals_received_;
    publish_state(gh, kBlendState);
    auto result = std::make_shared<Blend::Result>();
    const std::string refused = blend_refusal(*gh->get_goal());
    if (!refused.empty() || !blend_succeed_) {
      result->success = false;
      result->message = refused.empty()
                            ? "fake blend: no overlap"
                            : "fake blend: invalid legs: " + refused;
      gh->abort(result);
      return;
    }
    result->success = true;
    result->message = "fake blend ok";
    result->trajectory.header.frame_id = frame_id_;
    for (const auto &leg : gh->get_goal()->legs) {
      result->trajectory.joint_names = leg.joint_names;
      for (const auto &p : leg.points)
        result->trajectory.points.push_back(p);
    }
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  // header.frame_id on every plan. arm_mount by default: a planner that keeps
  // the contract. Unset mimics cuRobo, which leaves it empty.
  std::string frame_id_ = "arm_mount";
  std::vector<std::string> joint_names_ = kArmJointNames;
  rclcpp_action::Server<PlanToPose>::SharedPtr server_;
  rclcpp_action::Server<PlanToJoints>::SharedPtr joints_server_;
  rclcpp_action::Server<Blend>::SharedPtr blend_server_;
  bool succeed_;
  int n_points_;
  bool reject_;
  std::shared_future<void> gate_;
  std::shared_ptr<std::promise<void>> started_;
  bool reject_cancel_;
  bool bad_width_;
  std::atomic<int> pose_goals_received_{0};
  int fail_plan_number_ = 0;
  int gated_plan_number_ = 0;
  int empty_plan_number_ = 0;
  std::atomic<int> plans_executed_{0};
  bool blend_succeed_ = true;
  std::atomic<int> blend_goals_received_{0};
  std::vector<std::vector<double>> starts_seen_;
  std::vector<geometry_msgs::msg::Pose> pose_targets_seen_;
  std::vector<uint8_t> holds_seen_;
  std::vector<std::vector<double>> joint_targets_seen_;
  Blend::Goal last_blend_;
  mutable std::mutex seen_m_;
  std::vector<double> last_start_joints_;
  uint8_t last_hold_ = kNoHoldSeen;
  rammp_curobo_interfaces::msg::ApproachVia last_approach_via_;
};
} // namespace kinova_gen3_ros2::test
