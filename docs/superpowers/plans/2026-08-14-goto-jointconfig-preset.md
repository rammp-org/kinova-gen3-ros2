# GoToJointConfig + GoToPreset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two high-level joint-space actions — `GoToJointConfig` and `GoToPreset` — that plan collision-free via cuRobo `plan_to_joints` and execute through the existing `Supervisor` seam, on a shared lifecycle base that `GoToEEPose` is also refactored onto.

**Architecture:** Extract the proven `GoToEEPoseServer` plan→execute→settle→cancel lifecycle into a templated `PlannedMoveServer<ActionT>` base (settle-once logic in ONE place). Three thin concrete servers provide only `validate` + `start_plan`. `CuroboPlanClient` gains `plan_to_joints`. A preset registry (ROS params) maps names→joint configs.

**Tech Stack:** C++17, ROS2 Humble (rclcpp / rclcpp_action), ament_cmake, gtest. Reuses `GoalRouter`, `CuroboPlanClient`, the `Supervisor` `CommandSink` seam, `result_code::kPlanningFailed`.

**Design spec:** `docs/superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md`.

## Global Constraints

- **Builds aarch64-only on `abra`.** Build: `bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON`. Run a gtest binary: `ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; /tmp/kinova-ros2-ws/build/kinova_arm_ros2/<TESTBIN> --gtest_color=yes"'`.
- **Branch:** `feat/arm-goto-jointconfig-preset` (already created, **stacked on `feat/goto-ee-pose-curobo`**). Commit here. **No core change** (reuse `kPlanningFailed = -7`).
- **Settle-exactly-once is the crux invariant** — each goal terminals its `ServerGoalHandle` exactly once, never zero (hang) or twice (rclcpp throws). Mutex `m_` guards `goals_`; NO downstream ROS/sink call is made while holding `m_`. Preserve this verbatim when templating.
- **RT contract untouched** — no new code in `compute`/the executor cycle. Real-arm runs pin `--cpu 11` (RT fix `79d1050`; `make sim/real` do it).
- **Fail loud** — reject bad goals (wrong joint count, non-`base_link` frame, unknown preset) and malformed planner output (empty / non-7-wide trajectory → `kPlanningFailed`).
- Commit-message body ends with:
  ```
  Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01QoSEXbPQLpXMQAfw6ntqJv
  ```
- The refactor onto the base MUST be behavior-preserving for `GoToEEPose`: the existing `goto_ee_pose_integration_test` is the regression gate and must pass **unchanged**.

## cuRobo `plan_to_joints` contract (verified)

Action `/rammp_curobo/plan_to_joints`, type `rammp_curobo_interfaces/action/PlanToJoints`. Goal: `float64[] target_joints`, `float64[] start_joints` (empty ⇒ cuRobo reads our `/joint_states`). Result: `bool success`, `string message`, `trajectory_msgs/JointTrajectory trajectory` (time-parameterized, `joint_1..7`, full speed), `float64 planning_time`, `float64 goal_mismatch_rad`. Feedback: `string state`. Same shape as `plan_to_pose` apart from the goal + the extra result field.

---

### Task 1: interfaces — `GoToJointConfig.action` + `GoToPreset.action`

**Files:** Create `kinova_arm_interfaces/action/GoToJointConfig.action`, `kinova_arm_interfaces/action/GoToPreset.action`; Modify `kinova_arm_interfaces/CMakeLists.txt`.

**Interfaces produced:** two actions whose Result/Feedback are byte-identical to `GoToEEPose.action`; Goals differ.

- [ ] **Step 1: Create `GoToJointConfig.action`**
```
# Goal — move to a 7-joint configuration; cuRobo plans collision-free (plan_to_joints).
float64[7] target_joints   # rad, joint_1..joint_7
string     sender_id
---
# Result
int32   error_code   # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4,
                     # PREEMPTED=-6, PLANNING_FAILED=-7
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error
---
# Feedback
string  phase              # "planning" | "executing"
string  planner_state
float32 fraction_complete
trajectory_msgs/JointTrajectoryPoint actual
```

- [ ] **Step 2: Create `GoToPreset.action`** — identical Result/Feedback; Goal:
```
# Goal — move to a named joint configuration from the node's preset registry.
string preset_name
string sender_id
---
# (Result identical to GoToJointConfig)
int32   error_code
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error
---
# (Feedback identical to GoToJointConfig)
string  phase
string  planner_state
float32 fraction_complete
trajectory_msgs/JointTrajectoryPoint actual
```

- [ ] **Step 3: Register both in `kinova_arm_interfaces/CMakeLists.txt`** — add the two lines to the existing `rosidl_generate_interfaces(...)` alongside `GoToEEPose.action` (deps unchanged; these use only `trajectory_msgs`/`std_msgs`, already listed).

- [ ] **Step 4: Build + verify** — `bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON`; then `ros2 interface show kinova_arm_interfaces/action/GoToJointConfig` and `...GoToPreset` print the fields. Commit (`feat(interfaces): add GoToJointConfig + GoToPreset actions`).

---

### Task 2: `CuroboPlanClient::plan_to_joints` (+ DRY dispatch, type-erased cancel) + fake server

**Files:** Modify `include/kinova_arm_ros2/curobo_plan_client.h`, `src/curobo_plan_client.cpp`, `test/fake_curobo_server.h`, `test/curobo_plan_client_test.cpp`, `CMakeLists.txt` (test deps unchanged — `rammp_curobo_interfaces` already linked).

**Interfaces produced:** `void CuroboPlanClient::plan_to_joints(const std::vector<double>& target_joints, FeedbackCb, DoneCb)` → same `Outcome`. `cancel()` cancels whichever plan is in flight. `FakeCuroboServer` also serves `/rammp_curobo/plan_to_joints`.

- [ ] **Step 1: Extend `FakeCuroboServer`** (`test/fake_curobo_server.h`) to *also* host a `PlanToJoints` server, additively — reuse the same `succeed`/`n_points`/`reject`/`gate`/`started`/`reject_cancel` config so existing `PlanToPose` call sites are unchanged. Its `execute` returns the same canned 7-joint trajectory (add the `goal_mismatch_rad` field, `0.0` on success). Include `rammp_curobo_interfaces/action/plan_to_joints.hpp`.

- [ ] **Step 2: Write the failing `plan_to_joints` client tests** (`test/curobo_plan_client_test.cpp`) — mirror the four existing `plan()` tests for `plan_to_joints`: success returns a trajectory; abort → `ok=false`; rejected → `ok=false`; server-unavailable → `ok=false`. Use a fresh node + background-spin `MultiThreadedExecutor` + reentrant group, same harness as the existing tests. RED = link failure on `plan_to_joints`.

- [ ] **Step 3: Refactor `CuroboPlanClient` internals + add `plan_to_joints`.**
  In the header: add `void plan_to_joints(const std::vector<double>&, FeedbackCb, DoneCb);`, a second `rclcpp_action::Client<PlanToJoints>::SharedPtr client_joints_`, and replace `std::shared_ptr<GoalHandle> active_` with `std::function<void()> active_cancel_`. Add `using PlanToJoints = rammp_curobo_interfaces::action::PlanToJoints;` and the include.
  In the cpp: extract the current `plan()` body into a private templated helper:
```cpp
template <typename ClientT, typename GoalT>
void CuroboPlanClient::dispatch(ClientT& client, GoalT goal, FeedbackCb on_fb, DoneCb on_done) {
  auto once = std::make_shared<std::once_flag>();
  auto fire = [once, on_done](Outcome o) { std::call_once(*once, [&]{ on_done(std::move(o)); }); };
  if (!client->wait_for_action_server(std::chrono::milliseconds(200))) {
    fire({false, "cuRobo action server unavailable", {}}); return;
  }
  typename ClientT::element_type::SendGoalOptions opts;
  using GH = typename ClientT::element_type::GoalHandle;   // ClientGoalHandle<Action>
  opts.goal_response_callback = [this, fire, &client](typename GH::SharedPtr gh) {
    if (!gh) { fire({false, "cuRobo rejected plan goal", {}}); return; }
    std::lock_guard<std::mutex> l(m_);
    active_cancel_ = [c = client, gh]{ c->async_cancel_goal(gh); };
  };
  opts.feedback_callback = [on_fb](typename GH::SharedPtr,
      const std::shared_ptr<const typename GoalT::_target_type /* use the Feedback type */>){ /* see note */ };
  opts.result_callback = [this, fire](const typename GH::WrappedResult& wr) {
    { std::lock_guard<std::mutex> l(m_); active_cancel_ = nullptr; }
    Outcome o;
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result && wr.result->success) {
      o.ok = true; o.message = wr.result->message; o.trajectory = wr.result->trajectory;
    } else { o.ok = false;
      o.message = (wr.result && !wr.result->message.empty()) ? wr.result->message : "cuRobo plan failed"; }
    fire(std::move(o));
  };
  client->async_send_goal(goal, opts);
}
```
  *Note on the feedback callback:* both `PlanToPose::Feedback` and `PlanToJoints::Feedback` expose `state`. If the templated feedback type is awkward, keep `plan()`/`plan_to_joints()` as two ~8-line wrappers that build the goal and set `opts.feedback_callback = [on_fb](auto, auto fb){ if (on_fb) on_fb(fb->state); }` locally, and factor only the goal-response/result/cancel wiring. **Prefer whichever compiles cleanly with the least template gymnastics** — the goal is one exactly-once + one type-erased cancel, not maximal templating. `plan()` builds a `PlanToPose::Goal{target}`; `plan_to_joints()` builds a `PlanToJoints::Goal` with `target_joints` set and `start_joints` filled from the caller. `cancel()`:
```cpp
void CuroboPlanClient::cancel() {
  std::function<void()> c; { std::lock_guard<std::mutex> l(m_); c = active_cancel_; }
  if (c) c();
}
```

- [ ] **Step 4: Build + run `curobo_plan_client_test`** — all 8 (4 pose + 4 joints) pass. Confirm the existing pose tests still pass (the refactor is behavior-preserving). Commit (`feat(ros2): CuroboPlanClient plan_to_joints (+ type-erased cancel)`).

---

### Task 3: `PlannedMoveServer<ActionT>` base + refactor `GoToEEPose` onto it

**Files:** Create `include/kinova_arm_ros2/planned_move_server.h` (template, header-only) and `include/kinova_arm_ros2/joint_point.h` (shared `vec_to_point`). Rewrite `include/kinova_arm_ros2/goto_ee_pose_server.h` + delete/empty `src/goto_ee_pose_server.cpp` (logic moves into the template). Modify `CMakeLists.txt`. **`test/goto_ee_pose_integration_test.cpp` is UNCHANGED** and is the regression gate.

**Interfaces produced:** `template<class ActionT> class PlannedMoveServer : public kinova::interface::ActionServerPort` with ctor `(node, action_name, GoalRouter&, CuroboPlanClient&, cb_group)`, `set_command_sink`, and two pure-virtual hooks `validate(const ActionT::Goal&) -> std::optional<std::string>` and `start_plan(const ActionT::Goal&, CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)`.

- [ ] **Step 1: `joint_point.h`** — move the `vec_to_point` helper to a shared inline so the template can build `Feedback.actual`/`Result.final_error`:
```cpp
#pragma once
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_arm_ros2 {
inline trajectory_msgs::msg::JointTrajectoryPoint vec_to_point(const kinova::JointVec& v) {
  trajectory_msgs::msg::JointTrajectoryPoint p; p.positions.resize(kinova::kNumJoints);
  for (int i = 0; i < kinova::kNumJoints; ++i) p.positions[i] = v[i];
  return p;
}
}  // namespace kinova_arm_ros2
```
Update `message_mapping.cpp` to use this shared `vec_to_point` (drop its file-static copy) so there is one definition.

- [ ] **Step 2: Write `PlannedMoveServer<ActionT>`** — this is the current `goto_ee_pose_server.{h,cpp}` logic, verbatim, with `Action` → `ActionT`, the pose-specific bits replaced by the two hooks, and Feedback/Result built inline (fields are identical across actions). Full header:
```cpp
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/joint_point.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_arm_ros2 {

// Shared plan->execute->settle lifecycle for the high-level "go to a goal" actions.
// Concrete servers provide validate() + start_plan(); everything else (settle-once,
// cancel, width guard, GoalRouter registration) lives here, once.
template <class ActionT>
class PlannedMoveServer : public kinova::interface::ActionServerPort {
 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  PlannedMoveServer(rclcpp::Node::SharedPtr node, const std::string& action_name,
                    GoalRouter& router, CuroboPlanClient& planner,
                    rclcpp::CallbackGroup::SharedPtr cb_group)
      : node_(node), router_(router), planner_(planner) {
    using std::placeholders::_1; using std::placeholders::_2;
    server_ = rclcpp_action::create_server<ActionT>(
        node_, action_name,
        std::bind(&PlannedMoveServer::handle_goal, this, _1, _2),
        std::bind(&PlannedMoveServer::handle_cancel, this, _1),
        std::bind(&PlannedMoveServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
  }
  virtual ~PlannedMoveServer() = default;
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

  void publish_feedback(const kinova::interface::GoalId& id,
                        const kinova::interface::TrajectoryFeedback& fb) override {
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
      if (it == goals_.end()) return; gh = it->second.gh; }
    auto m = std::make_shared<typename ActionT::Feedback>();
    m->phase = "executing";
    m->fraction_complete = static_cast<float>(fb.fraction_complete);
    m->actual = vec_to_point(fb.actual);
    gh->publish_feedback(m);
  }
  void settle(const kinova::interface::GoalId& id,
              const kinova::interface::TrajectoryResult& r) override {
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
      if (it == goals_.end()) return; gh = it->second.gh; goals_.erase(it); }
    terminal(gh, r.error_code, r.error_string, r.final_error);
  }

 protected:
  // Hooks:
  virtual std::optional<std::string> validate(const typename ActionT::Goal& goal) = 0;
  virtual void start_plan(const typename ActionT::Goal& goal,
                          CuroboPlanClient::FeedbackCb on_fb,
                          CuroboPlanClient::DoneCb on_done) = 0;
  CuroboPlanClient& planner_;   // subclasses call planner_.plan / plan_to_joints
  rclcpp::Node::SharedPtr node_;

 private:
  using GoalId = kinova::interface::GoalId;
  static constexpr double kGotoPathTolRad = 0.35;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!sink_) return rclcpp_action::GoalResponse::REJECT;
    if (auto why = validate(*goal)) {
      RCLCPP_WARN(node_->get_logger(), "rejecting goal: %s", why->c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> gh) {
    const GoalId id = gh->get_goal_id();
    bool executing = false;
    { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
      if (it != goals_.end()) executing = it->second.executing; }
    if (executing) { if (sink_) sink_->on_trajectory_cancel(id); }
    else           { planner_.cancel(); }
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(std::shared_ptr<GoalHandle> gh) {
    const GoalId id = gh->get_goal_id();
    { std::lock_guard<std::mutex> l(m_); goals_[id] = Goal{gh, false}; }
    auto planning = std::make_shared<typename ActionT::Feedback>();
    planning->phase = "planning";
    gh->publish_feedback(planning);
    start_plan(*gh->get_goal(),
        [this, id](const std::string& state) {
          std::shared_ptr<GoalHandle> g;
          { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
            if (it == goals_.end()) return; g = it->second.gh; }
          auto f = std::make_shared<typename ActionT::Feedback>();
          f->phase = "planning"; f->planner_state = state;
          g->publish_feedback(f);
        },
        [this, id](CuroboPlanClient::Outcome o) { on_plan_done(id, std::move(o)); });
  }
  void on_plan_done(GoalId id, CuroboPlanClient::Outcome outcome) {
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
      if (it == goals_.end()) return; gh = it->second.gh; }
    using kinova::interface::result_code::kPreempted;
    using kinova::interface::result_code::kPlanningFailed;
    using kinova::interface::result_code::kInvalidGoal;
    if (!outcome.ok) {
      const bool cx = gh->is_canceling();
      settle_local(gh, cx ? kPreempted : kPlanningFailed,
                   cx ? "canceled during planning" : outcome.message);
      erase(id); return;
    }
    if (gh->is_canceling()) { settle_local(gh, kPreempted, "canceled during planning"); erase(id); return; }
    if (outcome.trajectory.points.empty()) {
      settle_local(gh, kPlanningFailed, "planner returned an empty trajectory"); erase(id); return; }
    for (const auto& p : outcome.trajectory.points)
      if (p.positions.size() != static_cast<size_t>(kinova::kNumJoints)) {
        settle_local(gh, kPlanningFailed, "planner returned malformed trajectory: point has " +
            std::to_string(p.positions.size()) + " positions, expected " +
            std::to_string(kinova::kNumJoints)); erase(id); return; }
    kinova::interface::TrajectoryGoal tg = to_trajectory_goal(outcome.trajectory);  // from message_mapping.h
    tg.path_tolerance = kinova::JointVec::Constant(kGotoPathTolRad);
    tg.sender_id = gh->get_goal()->sender_id;
    if (sink_->on_trajectory_goal(tg) != kinova::interface::GoalResponse::kAccept) {
      settle_local(gh, kInvalidGoal, "supervisor rejected planned trajectory"); erase(id); return; }
    { std::lock_guard<std::mutex> l(m_); auto it = goals_.find(id);
      if (it != goals_.end()) it->second.executing = true; }
    router_.register_owner(id, *this);
    sink_->on_trajectory_accepted(id, tg);
  }
  void settle_local(std::shared_ptr<GoalHandle> gh, int code, const std::string& msg) {
    terminal(gh, code, msg, kinova::JointVec::Zero());
  }
  void terminal(std::shared_ptr<GoalHandle> gh, int code, const std::string& msg,
                const kinova::JointVec& final_err) {
    auto out = std::make_shared<typename ActionT::Result>();
    out->error_code = code; out->error_string = msg; out->final_error = vec_to_point(final_err);
    if (gh->is_canceling())                                        gh->canceled(out);
    else if (code == kinova::interface::result_code::kSuccessful)  gh->succeed(out);
    else                                                           gh->abort(out);
  }
  void erase(const GoalId& id) { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }

  GoalRouter& router_;
  kinova::interface::CommandSink* sink_ = nullptr;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  struct Goal { std::shared_ptr<GoalHandle> gh; bool executing = false; };
  std::mutex m_;
  std::map<GoalId, Goal> goals_;
};
}  // namespace kinova_arm_ros2
```
  Note: `to_trajectory_goal(const trajectory_msgs::msg::JointTrajectory&)` is already declared in `message_mapping.h`; include it. Verify member init order (declare `node_`/`router_`/`planner_` consistently; the sketch splits them across `protected`/`private` — in the real file put all members in one section with a correct init list to avoid `-Wreorder`).

- [ ] **Step 3: Rewrite `GoToEEPoseServer` as a thin subclass.** New `goto_ee_pose_server.h`:
```cpp
#pragma once
#include "kinova_arm_ros2/planned_move_server.h"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
namespace kinova_arm_ros2 {
class GoToEEPoseServer
    : public PlannedMoveServer<kinova_arm_interfaces::action::GoToEEPose> {
 public:
  using Action = kinova_arm_interfaces::action::GoToEEPose;
  GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                   CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb)
      : PlannedMoveServer<Action>(node, "go_to_ee_pose", router, planner, cb) {}
 protected:
  std::optional<std::string> validate(const Action::Goal& g) override {
    if (g.target.header.frame_id != "base_link")
      return "frame_id '" + g.target.header.frame_id + "' != base_link";
    return std::nullopt;
  }
  void start_plan(const Action::Goal& g, CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) override {
    planner_.plan(g.target.pose, fb, done);
  }
};
}  // namespace kinova_arm_ros2
```
  Delete `src/goto_ee_pose_server.cpp`; drop the `goto_ee_pose_server` library's `.cpp` from `CMakeLists.txt` (it becomes header-only — the `goto_ee_pose_server` lib target either becomes an INTERFACE lib or is removed and consumers include the header directly + link `curobo_plan_client goal_router message_mapping`). Simplest: make `goto_ee_pose_server` an `INTERFACE` library (headers only) that links its deps, so existing `target_link_libraries(... goto_ee_pose_server)` in the node + test keep working. The `to_goto_feedback_msg`/`to_goto_result_msg` mappers in `message_mapping` become unused by the server (the base builds messages inline) — **leave them** (still covered by their unit tests) or note as orphaned; do not delete pre-existing code that isn't yours to remove unless it's now truly dead (mention it in the report).

- [ ] **Step 4: Build + run `goto_ee_pose_integration_test` UNCHANGED — must pass (regression gate).** Also run `curobo_plan_client_test`, `goal_router_test`, `message_mapping_test`. All green. This proves the base is behavior-preserving. Commit (`refactor(ros2): extract PlannedMoveServer base; GoToEEPose onto it`).

---

### Task 4: `GoToJointConfigServer` + integration test

**Files:** Create `include/kinova_arm_ros2/goto_joint_config_server.h`, `test/goto_joint_config_integration_test.cpp`; Modify `CMakeLists.txt`.

- [ ] **Step 1: Write the failing integration test** — mirror `goto_ee_pose_integration_test.cpp` (reuse `FakeCuroboServer` in `plan_to_joints` mode + a `FakeSupervisor` + a `GoToJointConfig` action client on a background-spin executor). Cases: (a) success → `error_code == kSuccessful`, `sup.got_goal`, submitted goal has 3 points, `control_mode == kPosition`; (b) fake abort → `kPlanningFailed`, `sup.got_goal == false`; (c) `target_joints` containing a non-finite value (e.g. NaN) → goal **rejected** (client `gh.accepted == false`). (`float64[7]` is type-enforced to 7 values, so wrong-count isn't reachable; finiteness is the real guard.) RED = link failure.

- [ ] **Step 2: Implement the thin server:**
```cpp
#pragma once
#include "kinova_arm_ros2/planned_move_server.h"
#include "kinova_arm_interfaces/action/go_to_joint_config.hpp"
namespace kinova_arm_ros2 {
class GoToJointConfigServer
    : public PlannedMoveServer<kinova_arm_interfaces::action::GoToJointConfig> {
 public:
  using Action = kinova_arm_interfaces::action::GoToJointConfig;
  GoToJointConfigServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                        CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb)
      : PlannedMoveServer<Action>(node, "go_to_joint_config", router, planner, cb) {}
 protected:
  std::optional<std::string> validate(const Action::Goal& g) override {
    // target_joints is float64[7] -> fixed size, but guard finiteness.
    for (double q : g.target_joints)
      if (!std::isfinite(q)) return "target_joints contains a non-finite value";
    return std::nullopt;
  }
  void start_plan(const Action::Goal& g, CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) override {
    planner_.plan_to_joints(std::vector<double>(g.target_joints.begin(), g.target_joints.end()), fb, done);
  }
};
}  // namespace kinova_arm_ros2
```
  (Note: `float64[7]` maps to `std::array<double,7>`, always size 7, so the "size" check is unnecessary — finiteness is the real guard. If the plan later switches the field to unbounded `float64[]`, add a `== 7` check. Register the test target + link `goto_ee_pose_server`/base deps in `CMakeLists.txt`.)

- [ ] **Step 3: Build + run `goto_joint_config_integration_test` — 3/3 pass.** Commit (`feat(ros2): GoToJointConfig via cuRobo plan_to_joints`).

---

### Task 5: `GoToPresetServer` + preset registry + integration test

**Files:** Create `include/kinova_arm_ros2/goto_preset_server.h`, `test/goto_preset_integration_test.cpp`; Modify `CMakeLists.txt`.

- [ ] **Step 1: Write the failing integration test** — construct `GoToPresetServer` with an explicit registry (constructor takes a `std::map<std::string,std::vector<double>>` so the test can inject `{"home", {…7…}}` without ROS params). Cases: (a) `preset_name="home"` → success, submitted; (b) `preset_name="nope"` → goal **rejected**. RED = link failure.

- [ ] **Step 2: Implement the server** (registry injected; the node builds it from params in Task 6):
```cpp
#pragma once
#include <map>
#include <vector>
#include "kinova_arm_ros2/planned_move_server.h"
#include "kinova_arm_interfaces/action/go_to_preset.hpp"
namespace kinova_arm_ros2 {
class GoToPresetServer
    : public PlannedMoveServer<kinova_arm_interfaces::action::GoToPreset> {
 public:
  using Action = kinova_arm_interfaces::action::GoToPreset;
  GoToPresetServer(rclcpp::Node::SharedPtr node, GoalRouter& router, CuroboPlanClient& planner,
                   rclcpp::CallbackGroup::SharedPtr cb,
                   std::map<std::string, std::vector<double>> registry)
      : PlannedMoveServer<Action>(node, "go_to_preset", router, planner, cb),
        registry_(std::move(registry)) {}
 protected:
  std::optional<std::string> validate(const Action::Goal& g) override {
    if (!registry_.count(g.preset_name)) return "unknown preset '" + g.preset_name + "'";
    return std::nullopt;
  }
  void start_plan(const Action::Goal& g, CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) override {
    planner_.plan_to_joints(registry_.at(g.preset_name), fb, done);
  }
 private:
  std::map<std::string, std::vector<double>> registry_;
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 3: Build + run `goto_preset_integration_test` — 2/2 pass.** Commit (`feat(ros2): GoToPreset (named joint-config registry)`).

---

### Task 6: Bring-up wiring + preset params + startup verification

**Files:** Modify `src/bringup_node.cpp`, `CMakeLists.txt` (link the two new header-only servers' deps — if the servers are header-only, the node just needs the includes + existing links; add a small `load_presets` helper here or a free function).

- [ ] **Step 1: Preset registry from params.** Add a helper in `bringup_node.cpp`:
```cpp
static std::map<std::string, std::vector<double>> load_presets(rclcpp::Node& node) {
  std::map<std::string, std::vector<double>> reg;
  auto names = node.declare_parameter<std::vector<std::string>>("preset_names", {"home"});
  const std::vector<double> home = {0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571};  // cuRobo retract
  for (const auto& n : names) {
    auto q = node.declare_parameter<std::vector<double>>("presets." + n, n == "home" ? home : std::vector<double>{});
    if (q.size() == static_cast<size_t>(kinova::kNumJoints)) reg[n] = q;
    else RCLCPP_WARN(node.get_logger(), "preset '%s' has %zu joints (need 7) — skipping", n.c_str(), q.size());
  }
  return reg;
}
```

- [ ] **Step 2: Construct the two new servers** next to `goto_server` (same `router`, `planner`, `cb_group`), and `set_command_sink(&sup)` on each:
```cpp
  kinova_arm_ros2::GoToJointConfigServer jc_server(node, router, planner, cb_group);
  kinova_arm_ros2::GoToPresetServer preset_server(node, router, planner, cb_group, load_presets(*node));
  ...
  goto_server.set_command_sink(&sup);
  jc_server.set_command_sink(&sup);
  preset_server.set_command_sink(&sup);
```
  Declaration order: all servers before `sup` (already the case for `goto_server`); they must outlive `sup` — place them together. Update the startup `RCLCPP_INFO` to list all four actions.

- [ ] **Step 3: Build; launch (sim) on abra; verify `ros2 action list`** shows `/go_to_ee_pose`, `/go_to_joint_config`, `/go_to_preset`, `/execute_joint_trajectory`. Then run `scripts/abra_e2e_sim.sh` — `ExecuteJointTrajectory` regression must stay `success_case=0 divergence_case=0`. `pkill -TERM -f kinova_arm_node`, verify gone. Commit (`feat(ros2): host GoToJointConfig + GoToPreset in kinova_arm_node`).

---

### Task 7: Docs + client tooling

**Files:** Modify `docs/guide-goto-ee-pose.md` (rename/extend to cover the tier, or add a sibling `docs/guide-goto-actions.md`), `README.md`; keep `test/send_goto_pose_sequence.py` (already present) and add a tiny `test/send_goto_preset.py` (optional, one-goal preset client).

- [ ] **Step 1: Document the three actions** — one guide covering `GoToEEPose` / `GoToJointConfig` / `GoToPreset`: what each takes, the shared two-node cuRobo bring-up, result codes, cancel behavior, full-speed safety note, and how to configure presets via `preset_names` + `presets.<name>` params. Link the design spec.
- [ ] **Step 2: README** — list the three high-level actions the node hosts + guide pointer.
- [ ] **Step 3: Commit** (`docs(ros2): GoToJointConfig/GoToPreset guide + README`).

---

## Validation milestones (operational, attended — after Task 7)

- **Sim e2e with real cuRobo:** on abra, real `rammp_curobo` planner + `kinova_arm_node --sim`; send a `GoToJointConfig` (e.g. small delta from current) and a `GoToPreset home` — expect SUCCESSFUL, arm never moves (SimTransport).
- **Attended real-arm:** `make real IP=… CORE_REF=feat/planning-failed-result-code` (pins `--cpu 11`); small/near `GoToJointConfig`, then `GoToPreset home`; e-stop in hand, per `docs/on-robot-runbook.md`.

## PR

Stacked PR on `feat/goto-ee-pose-curobo` (PR #2). Depends on core PR #12 (`kPlanningFailed`) + PR #2 merging first. No new core change.

## Self-review notes (address during implementation)

- The templated base sketch splits members across `protected`/`private` for readability — in the real header, order members + the constructor init-list to avoid `-Wreorder`, and keep `node_`/`router_`/`planner_`/`sink_`/`server_`/`m_`/`goals_` consistent.
- If the header-only `goto_ee_pose_server` INTERFACE-lib conversion fights CMake, the fallback is a 1-line `.cpp` per server that just `#include`s its header (keeps the existing `add_library(... src/*.cpp)` shape). Prefer whichever is least disruptive to the existing `CMakeLists.txt`.
- Keep `message_mapping`'s `to_goto_feedback_msg`/`to_goto_result_msg` even if the base no longer calls them (their tests still pass); note them as orphaned in the final report rather than deleting pre-existing code.
