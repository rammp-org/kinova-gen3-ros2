# GoToJointConfig + GoToPreset — high-level joint action tier (design)

**Date:** 2026-08-14
**Status:** Design (approved by user; ready to plan/implement in a fresh session)
**Repo:** `rammp-org/kinova_gen3_ros2` — **stacked on `feat/goto-ee-pose-curobo` (PR #2)**.
Core is untouched (reuses `result_code::kPlanningFailed = -7`).

## What this is

The remaining two high-level actions from the action-tier handoff
(`docs/2026-08-14-curobo-action-tier-handoff.md` §2): **`GoToJointConfig`** and
**`GoToPreset`**, bundled into one PR stacked on the just-shipped `GoToEEPose`.
Both delegate collision-free planning to the external cuRobo node and execute the
returned trajectory through the same `Supervisor` `CommandSink` seam.

- **`GoToJointConfig`** — "move to this 7-joint configuration," collision-aware via
  cuRobo **`/rammp_curobo/plan_to_joints`** (NOT a planner-bypass direct move — the
  value-add over `ExecuteJointTrajectory` is exactly the collision-aware plan).
- **`GoToPreset`** — "move to a named configuration." A preset is *just a named joint
  config*: the server resolves `preset_name → 7 joints` from a registry, then plans
  it via the same `plan_to_joints` path. Value-add is the named registry.

Both are close parallels to `GoToEEPose`; the only real differences are how the
target is derived and that they call `plan_to_joints` instead of `plan_to_pose`.

## The decision that shapes the structure (user-approved)

`GoToEEPose`, `GoToJointConfig`, and `GoToPreset` all run the **identical
plan→execute→settle lifecycle** — the ~90 lines of concurrency-sensitive code in
`goto_ee_pose_server.cpp` (planning feedback → `on_plan_done` with the
`is_canceling` guard + width guard + Supervisor submit + `GoalRouter` registration
→ `settle_local`/`settle`/`publish_feedback`, with the **settle-exactly-once**
invariant). Rather than copy it per action, we **factor it into one shared base and
put all three actions on it** (the user chose full unification, including
refactoring the existing `GoToEEPose` — the churn to PR #2 is accepted because it
keeps the settle-once logic auditable in ONE place, which matters most for this
driver).

## Architecture

### Shared lifecycle base — `PlannedMoveServer<ActionT>` (new, header-only template)

Owns everything common to a "plan → execute → settle" action, templated on the ROS
action type. It holds the `rclcpp_action::Server<ActionT>`, the `goals_` map +
mutex, the `CommandSink*`, and the `GoalRouter&`, and implements `ActionServerPort`
for its own goals. It carries **verbatim** the proven `GoToEEPoseServer` logic:
`handle_goal`/`handle_accepted`/`handle_cancel`, `on_plan_done` (is_canceling guard,
empty/width guard → `kPlanningFailed`, `on_trajectory_goal` submit, `register_owner`,
`on_trajectory_accepted`), `settle_local`, `settle`, `publish_feedback`. Mutex
discipline (no downstream call under the lock) and settle-once are preserved exactly.

Two hooks the concrete servers provide:
- `std::optional<std::string> validate(const ActionT::Goal&)` — return a rejection
  reason (fail-loud) or `nullopt`. Called in `handle_goal`.
- `void start_plan(const ActionT::Goal&, CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)`
  — resolve the target and call the right planner method, forwarding the two
  callbacks. Called in `handle_accepted`.

Because all three actions share an **identical `Result`/`Feedback` layout** (only the
`Goal` differs — see interfaces below), the base sets `phase`/`planner_state`/
`fraction_complete`/`actual`/`error_code`/`error_string`/`final_error` uniformly via
`typename ActionT::Feedback`/`Result` (a small `vec_to_point` helper moves into a
shared header). `sender_id` is read uniformly as `gh->get_goal()->sender_id`
(present on all three goals). The generous `kGotoPathTolRad = 0.35` default stays.

### Concrete servers (thin — ~validate + start_plan)

- `GoToEEPoseServer : PlannedMoveServer<GoToEEPose>` — `validate`: `frame_id ==
  base_link`; `start_plan`: `planner.plan(goal.target.pose, …)`. **Refactored onto
  the base; behavior identical** — the existing `goto_ee_pose_integration_test` is
  the regression gate and must still pass unchanged.
- `GoToJointConfigServer : PlannedMoveServer<GoToJointConfig>` — `validate`:
  `target_joints.size() == 7` and all finite; `start_plan`:
  `planner.plan_to_joints(goal.target_joints, …)`.
- `GoToPresetServer : PlannedMoveServer<GoToPreset>` — holds a `const
  std::map<std::string, std::vector<double>>` registry (from ROS params);
  `validate`: `preset_name` present in the registry; `start_plan`:
  `planner.plan_to_joints(registry.at(goal.preset_name), …)`.

### `CuroboPlanClient` — add `plan_to_joints` (additive, internals DRY'd)

Gains a second action client for **`/rammp_curobo/plan_to_joints`**
(`rammp_curobo_interfaces/action/PlanToJoints`: Goal `float64[] target_joints,
float64[] start_joints`; Result `bool success, string message,
trajectory_msgs/JointTrajectory trajectory, float64 planning_time, float64
goal_mismatch_rad`; Feedback `string state`). New method:
`void plan_to_joints(const std::vector<double>& target_joints, FeedbackCb, DoneCb)`
returning the same `Outcome{ok,message,trajectory}` (start_joints filled from the
arm node's measured state; the empty form would make cuRobo
reads our `/joint_states`).

The existing `plan()` and the new `plan_to_joints()` share a **private templated
`dispatch<...>` helper** carrying the once-flag / `wait_for_action_server` /
`send_goal` / callback wiring (currently inline in `plan()`), so the exactly-once
guarantee lives in one place for both. **`cancel()` becomes type-erased**: instead of
a single `active_` `ClientGoalHandle<PlanToPose>`, store a
`std::function<void()> active_cancel_` set in each goal-response callback (bound to the
right client's `async_cancel_goal`) and cleared on result — so `cancel()` cancels
whichever plan (pose or joints) is in flight. One-plan-at-a-time holds as before.

### Preset registry (ROS params)

Node declares `preset_names` (string[], default `["home"]`) and, per name, a
`presets.<name>` double[7] param. Default: `presets.home =
[0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571]` (cuRobo's retract/home from
`robot_gen3_2f85.yaml`). `GoToPresetServer` reads these at construction into its
`const` registry; an unknown `preset_name` is rejected fail-loud in `validate`.

### Interfaces (new `.action` files, `kinova_gen3_interfaces`)

`GoToJointConfig.action` and `GoToPreset.action`. **Result and Feedback blocks are
byte-identical to `GoToEEPose.action`** (so the templated base sets them uniformly):
```
# Result
int32   error_code    # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4,
                      # PREEMPTED=-6, PLANNING_FAILED=-7
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error
# Feedback
string  phase              # "planning" | "executing"
string  planner_state
float32 fraction_complete
trajectory_msgs/JointTrajectoryPoint actual
```
Goals:
- `GoToJointConfig.action` Goal: `float64[7] target_joints`, `string sender_id`.
- `GoToPreset.action` Goal: `string preset_name`, `string sender_id`.
No `geometry_msgs` needed for these two; reuse the existing deps.

### Bring-up

Construct `GoToJointConfigServer` and `GoToPresetServer` on the **same reentrant
callback group** as `GoToEEPoseServer`, sharing the one `CuroboPlanClient` and the
`GoalRouter` (still default-fallthrough to `Ros2Backend`); `set_command_sink(&sup)`
on each. Declare the preset params. Update the startup log to list all action
servers. `MultiThreadedExecutor` + the RT-core `--cpu` pinning are already in place
(RT fix `79d1050`) — nothing changes there; **no new RT-path code**.

## Data flow (both new actions)

```
client ─ GoToJointConfig(target_joints) / GoToPreset(name) ─▶ concrete server
   validate ─▶ (Preset: name→joints) ─▶ PlannedMoveServer base
   base ─ start_plan ─▶ CuroboPlanClient.plan_to_joints ─▶ /rammp_curobo/plan_to_joints
                        (cuRobo reads our /joint_states) ◀─ JointTrajectory (full speed)
   base: is_canceling? width guard? ─▶ CommandSink.on_trajectory_goal/accepted ─▶ Supervisor
   Supervisor ─▶ GoalRouter.publish_feedback/settle ─▶ base ─▶ action feedback/result
```
Planning failure / unknown preset / bad joints short-circuit to `settle_local`
(`kPlanningFailed`/`kInvalidGoal`), no Supervisor submit — same as `GoToEEPose`.

## Testing

- **`PlannedMoveServer` base:** validated by the refactored `GoToEEPose` integration
  test (unchanged — the regression gate) plus the two new integration tests below.
- **`CuroboPlanClient::plan_to_joints`:** extend `FakeCuroboServer` to *also* serve
  `/rammp_curobo/plan_to_joints` (additive), then client tests for
  success/abort/reject/unavailable on the joints path (mirror the pose-path tests).
- **`GoToJointConfig` integration:** fake `plan_to_joints` → SUCCESSFUL (position
  goal submitted); fake abort → `PLANNING_FAILED`; bad target (wrong joint count) →
  rejected/`INVALID_GOAL`.
- **`GoToPreset` integration:** known preset (`home`) → SUCCESSFUL; unknown preset →
  rejected. Registry-from-params covered by the bring-up path.
- **Regression:** the existing `ExecuteJointTrajectory` e2e-sim + `GoToEEPose`
  integration tests must stay green after the refactor.
- All on abra (`scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2 --cmake-args
  -DBUILD_TESTING=ON`); real-arm runs pin **`--cpu 11`** (per RT fix `79d1050`;
  `make sim/real` do this automatically).

## Out of scope (future specs — NOT this PR)

- **`set_gains` service** — its Supervisor hook (`on_set_gains`) is a no-op stub;
  wiring a ROS service to a stub is not useful. Needs core gain-application work first.
- **`query_state` service** and a **JointState/loop-timing diagnostic topic** — cheap
  ROS2 fast-follows, but a different category (services/topics, not actions); left out
  to keep this PR the "action tier."
- **Streaming / topic tier** (reactive closed-loop) — highest RT-safety bar; needs its
  own safety-contract spec first.
- **Cross-tier arbitration** — hooks only (`sender_id`); its own spec.
