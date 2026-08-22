# GoToEEPose — cuRobo-backed high-level action tier (design)

**Date:** 2026-08-14
**Status:** Design (pending user review)
**Repos:** `rammp-org/kinova_arm_ros2` (this repo — the new action + cuRobo client)
and a one-line core touch in `rammp-org/kinova-gen3-driver` (a new result code).

## What this is

The first high-level, goal-oriented action on top of the shipped
`ExecuteJointTrajectory` frontend: **`GoToEEPose`** — "move the tool to this
base-frame pose." Collision-free planning is delegated to the **already-existing
external cuRobo node** (`ChrissCox/RAMMP-CuRobo`); our node is a *client* of its
planning action and executes the returned trajectory through the **same
`Supervisor` seam** the `ExecuteJointTrajectory` handler already uses.

This realizes the tier locked in the handoff
(`docs/2026-08-14-curobo-action-tier-handoff.md`, §2): EE-pose goals go through
cuRobo; pre-planned `ExecuteJointTrajectory` goals still bypass the planner; **one
node** of ours hosts both action servers and is a cuRobo client; high-level
handlers feed the planned trajectory into the `Supervisor` internally (no
self-call of `ExecuteJointTrajectory` over ROS).

**Scope of this round: `GoToEEPose` only, end-to-end to an attended real-arm run.**
`GoToJointConfig` and `GoToPreset` are deferred — both are largely degenerate
`ExecuteJointTrajectory`/`plan_to_joints` variants and add no new seam.

## The external contract we design against (cuRobo, verified from the repo)

cuRobo exposes a ROS **action** — planning only, it never moves the arm:

- **Action:** `/rammp_curobo/plan_to_pose`, type `rammp_curobo_interfaces/action/PlanToPose`
- **Goal:** `geometry_msgs/Pose target` (bare `Pose`, **base_link** frame, quaternion
  xyzw) + `float64[] start_joints` (empty ⇒ cuRobo reads `/joint_states` itself,
  2.0 s staleness cutoff; else plan from the given `joint_1..7` config)
- **Result:** `bool success`, `string message`, `trajectory_msgs/JointTrajectory
  trajectory` (time-parameterized, `joint_1..7`, dense `interpolation_dt = 0.02 s`
  waypoints, **planned at full speed**; point *k* stamped at `(k+1)·dt`),
  `float64 planning_time`
- **Feedback:** `string state`
- ee_link is `tool_frame` (≈ 2F-85 fingertip), baked into the cuRobo robot config —
  **not** a per-request field. cuRobo owns its own collision world (swap via
  `/rammp_curobo/set_world` — out of scope here). **One plan at a time**: a
  concurrent request is aborted `success=false, "planner busy — one plan at a time"`.
- Failure signaling is `success=false` + aborted goal + `message`; **no numeric
  error code**.

**The seam fits for free:** our node already publishes `/joint_states` (~100 Hz,
`SensorDataQoS`, `joint_1..7`), which is exactly what cuRobo reads for its start
config. **Superseded:** we now send `start_joints` explicitly, filled from the
arm node's own measured state (`CommandSink::on_query_state`). Leaving it empty
makes the planner subscribe to `/joint_states` and accept a start state up to
2 s old, which can differ from the one we execute from.
We depend only on `rammp_curobo_interfaces` — the deliberately dependency-free IDL
package (no GPU stack) — for the `PlanToPose` type.

## Decisions locked (this design)

1. **Execute cuRobo's trajectory at full speed, as-is.** No `speed_scale` field, no
   time rescaling — the returned `time_from_start` feeds straight through. *Operational
   consequence (recorded, not mitigated in code):* the first real-arm `GoToEEPose`
   runs at cuRobo full speed, so the attended bring-up (Milestone C) picks a small /
   near target, e-stop in hand, per `docs/on-robot-runbook.md`.
2. **Position control only (v1).** The collision-free plan is tracked in
   `JointPositionMode`. The `GoToEEPose` message carries no control-mode / gains
   fields; impedance is a clean fast-follow if a contact task needs it.
3. **`GoToEEPose` goal is a `geometry_msgs/PoseStamped`, required frame `base_link`.**
   Fail loud (reject the goal) on any other `frame_id`; no TF this round. Frame-aware
   and forward-compatible with adding a TF transform later. We strip to the bare
   `Pose` cuRobo wants before forwarding.

## New interface — `kinova_arm_interfaces/action/GoToEEPose.action`

```
# Goal
geometry_msgs/PoseStamped target   # tool (tool_frame) pose; frame_id MUST be base_link
string sender_id                   # arbitration hook, mirrors ExecuteJointTrajectory
---
# Result
int32  error_code    # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4,
                     # PREEMPTED=-6, PLANNING_FAILED=-7
string error_string  # cuRobo's message on PLANNING_FAILED; executor detail otherwise
trajectory_msgs/JointTrajectoryPoint final_error   # joint-space final error
---
# Feedback
string  phase              # "planning" | "executing"
string  planner_state      # cuRobo feedback 'state' relayed during planning
float32 fraction_complete  # execution progress (0 while planning)
trajectory_msgs/JointTrajectoryPoint actual   # live measured q during execution
```

`error_code` reuses the numeric values of the core `result_code` enum and adds
**`PLANNING_FAILED = -7`**. The existing codes mirror `control_msgs/
FollowJointTrajectory` (`-1`/`-4`/`-5`), whose `-2`/`-3` mean `INVALID_JOINTS`/
`OLD_HEADER_TIMESTAMP` — so `PLANNING_FAILED` takes the next free *custom* slot after
this codebase's own `PREEMPTED=-6`, rather than overloading an FJT-reserved value.
`final_error` is joint-space only — a Cartesian pose error would need FK on the ROS
side and buys nothing for v1.

## Core touch — add `PLANNING_FAILED` to `result_code`

The only core change. In `include/kinova_lowlevel/interface/value_types.h`:

```cpp
namespace result_code {
  constexpr int kSuccessful=0, kInvalidGoal=-1, kPathToleranceViolated=-4,
                kGoalToleranceViolated=-5, kPreempted=-6, kPlanningFailed=-7;
}
```

`PLANNING_FAILED` is produced **only by our ROS server** (planning happens outside
the Supervisor). The Supervisor never emits it; the enum entry just gives the whole
stack one authoritative name and keeps the `.action` comment honest. This is the
sole justification for touching the core (handoff §7: "keep the core untouched
unless a new bug demands it" — this is a required new code, not a refactor).

## Components (this repo)

The rclcpp-including surface grows from one unit to three ROS-facing **adapters**.
The invariant's *spirit* holds — rclcpp stays out of the core and out of the
portable value-type logic — so we restate it precisely: **the rclcpp-including units
are `Ros2Backend`, `GoToEEPoseServer`, and `CuroboPlanClient`; nothing else.**

### `CuroboPlanClient` (new — rclcpp action client)

The only unit that knows cuRobo exists. Wraps a `rclcpp_action::Client<PlanToPose>`.

- API (roughly): `plan(const geometry_msgs::Pose& target, PlanCallbacks cbs)`
  returning a cancellable handle; `cbs` = `{on_feedback(state), on_done(PlanOutcome)}`.
- `PlanOutcome` is a small plain struct: `{bool ok; std::string message;
  trajectory_msgs::JointTrajectory trajectory;}` — cuRobo's `success`/`message`/
  `trajectory`, plus a synthesized failure for the *aborted*, *rejected*, and
  *server-unavailable* cases (all surface as `ok=false` with a message).
- Sends `start_joints` **filled** from `CommandSink::on_query_state()`, so the
  planner never sources the arm's state itself. `handle_goal` refuses the goal
  outright if no measurement has arrived yet (`stamp_s <= 0`), rather than
  sending a zeroed configuration that would read as a real one.
- Runs on a **reentrant callback group** so its async round-trip never blocks the
  action servers (see Threading).

### `GoToEEPoseServer` (new — rclcpp action server + `ActionServerPort`)

Hosts the `GoToEEPose` server and owns the per-goal state machine. Holds a
`CommandSink*` (injected, like `Ros2Backend`) and a `GoalRouter&` (below).

- `handle_goal`: reject if no sink; **reject unless `target.header.frame_id ==
  "base_link"`** (fail loud); else accept.
- `handle_accepted(gh)`: store the handle under its UUID; publish `phase="planning"`
  feedback; call `CuroboPlanClient::plan(strip_to_pose(target), …)`.
- On plan **failure** (`ok=false`): settle the `GoToEEPose` goal
  `PLANNING_FAILED`, `error_string = cuRobo message`; unregister.
- On plan **success**: build a `kinova::interface::TrajectoryGoal` from the returned
  `JointTrajectory` (positions + `time_from_start` → `JointWaypoint{q, t_s}`;
  `control_mode = kPosition`; `preemption = kLatestWins`; a generous default
  `path_tolerance`, see below; `sender_id` copied). Register ownership of this UUID
  with the `GoalRouter`, then drive the **same seam** `Ros2Backend` uses:
  `sink_->on_trajectory_goal(goal)` → if `kAccept`, `sink_->on_trajectory_accepted(
  uuid, goal)`; if `kReject`, settle `INVALID_GOAL`.
- Implements `ActionServerPort` **for its own goals**: `publish_feedback(uuid, fb)`
  → `GoToEEPose` feedback (`phase="executing"`, `fraction_complete`, `actual`);
  `settle(uuid, result)` → map `result_code` → `error_code`, then
  `succeed()`/`abort()`/`canceled()` on the handle (mirrors `Ros2Backend::settle`).
- `handle_cancel(gh)`: if **still planning**, cancel the cuRobo goal via the plan
  handle (→ settle `PREEMPTED`); if **executing**, `sink_->on_trajectory_cancel(
  uuid)` (Supervisor settles `kPreempted` → `PREEMPTED`/`canceled()`).

The `GoToEEPose` UUID is reused as the Supervisor `GoalId` — unique per goal, no
collision with `ExecuteJointTrajectory` UUIDs.

### `GoalRouter` (new — tiny `ActionServerPort` demux, no rclcpp)

The one structural addition that makes two action servers share one Supervisor.
The Supervisor takes a **single** `ActionServerPort&`; today `Ros2Backend` is it.
`GoalRouter` becomes that single port and fans out by `GoalId`:

- Holds `std::map<GoalId, ActionServerPort*>` (mutex-guarded — registered from the
  rclcpp threads, read from the Supervisor sampler thread), i.e. **who owns this
  trajectory**.
- `register_owner(GoalId, ActionServerPort&)` / auto-clear on `settle`.
- `publish_feedback(id, fb)` / `settle(id, result)` → look up the owner and delegate;
  `settle` erases the entry after delegating.
- Depends only on `interface/ports.h` + `value_types.h` (`GoalId`); **no rclcpp**,
  so it stays out of the ROS-adapter set and could later move to core if reused.

Both `Ros2Backend` (its existing `ExecuteJointTrajectory` server) and
`GoToEEPoseServer` register ownership at submit/accept time and each keep their own
`ActionServerPort` `settle`/`publish_feedback` logic unchanged. `Ros2Backend`'s
change is minimal: one `register_owner(uuid, *this)` call in `handle_accepted`; it
otherwise keeps its handle map and settle path exactly as-is.

*Alternatives considered:* (A) fold cuRobo + `GoToEEPose` into `Ros2Backend` —
rejected (god-node drift the handoff §3 warns against). (B) keep `Ros2Backend` the
sole `ActionServerPort` and register external completion callbacks into it —
rejected (couples `GoToEEPoseServer` to `Ros2Backend` and bloats it). The
`GoalRouter` keeps each adapter single-purpose, which is the codebase's stated
design value.

## Data flow

```
client ── GoToEEPose(PoseStamped target, base_link) ──▶ GoToEEPoseServer
  validate frame ─▶ CuroboPlanClient.plan(Pose)
                      └▶ /rammp_curobo/plan_to_pose  (cuRobo reads our /joint_states)
                      ◀── success + trajectory_msgs/JointTrajectory (full speed)
  build TrajectoryGoal (position, full-speed timing)
  GoalRouter.register_owner(uuid, GoToEEPoseServer)
  CommandSink.on_trajectory_goal / on_trajectory_accepted   ─▶ Supervisor
                                                                 └▶ sampler → RtExecutor (1 kHz) → Sim/Kortex
  Supervisor ──▶ GoalRouter.publish_feedback / settle(uuid) ──▶ GoToEEPoseServer
  ─▶ GoToEEPose feedback (executing, fraction, actual) / result (error_code, final_error)
```

Planning failure short-circuits after the cuRobo call: no Supervisor submit, settle
`PLANNING_FAILED` directly. This keeps the two failure classes distinct
(`PLANNING_FAILED` vs `PATH_TOLERANCE_VIOLATED`), per handoff §3.

## Threading (handoff §4 item 4)

cuRobo planning is a **tens-to-hundreds-of-ms** async round-trip; blocking the ROS
executor on it would starve the `ExecuteJointTrajectory` server and feedback. So:

- Bring-up switches from `SingleThreadedExecutor` to a **`MultiThreadedExecutor`**.
- The cuRobo client and both action servers run on a **reentrant callback group** so
  a plan-in-flight callback and the `ExecuteJointTrajectory` callbacks make progress
  concurrently.
- **Nothing new touches the RT thread.** Feeding the trajectory into the Supervisor
  is the same lock-free-inbox hand-off as today (`q_mtx_` + `inbox_.push_back`); the
  sampler/pump/RT loop are unchanged. The RT-safety contract is untouched — the added
  work is all non-RT rclcpp.

## Path-tolerance default (a full-speed consideration)

cuRobo's collision-free path is dense (0.02 s) and executed at full speed, so brief
tracking lag is expected. A tight Supervisor divergence guard would risk a false
`PATH_TOLERANCE_VIOLATED` (the failure mode class that produced the fixed core bug
`dd3e57e`). v1 sets a **generous default** `path_tolerance` for `GoToEEPose`-submitted
goals (a single constant in `GoToEEPoseServer`, ~0.35 rad; tunable later, not a
message field) rather than disabling the guard outright. Measured on the attended run.

## Dependencies & build

- `kinova_arm_interfaces` gains `GoToEEPose.action` and a **new `geometry_msgs`
  dependency** (it currently depends on `builtin_interfaces`, `std_msgs`,
  `trajectory_msgs`, `control_msgs`, `action_msgs` — `PoseStamped` needs
  `geometry_msgs` added to `package.xml` + `CMakeLists.txt`).
- `kinova_arm_ros2` build-depends on **`rammp_curobo_interfaces`**; add it to
  `kinova_arm.repos` (source-built alongside the core, GPU stack NOT required).
- New targets: `curobo_plan_client`, `goto_ee_pose_server`, `goal_router` libs,
  linked into `kinova_arm_node`. `Ros2Backend` + Supervisor wiring updated in
  `bringup_node.cpp`: construct the `GoalRouter`, pass it as the Supervisor's
  `ActionServerPort`, construct `CuroboPlanClient` + `GoToEEPoseServer`, inject the
  `CommandSink` + router into both servers.

## Testing strategy

- **Mapping unit test (no ROS):** `JointTrajectory` → `TrajectoryGoal` — positions,
  `time_from_start` → `t_s`, 7-DOF size check, control-mode = position. Pure, fast.
- **`GoalRouter` unit test (no ROS):** register two ids to two fake
  `ActionServerPort`s; assert `publish_feedback`/`settle` reach the right owner and
  `settle` clears ownership.
- **Sim integration (rclcpp, fake cuRobo):** a **fake `PlanToPose` action server**
  returns a canned `JointTrajectory`; `GoToEEPoseServer` + a `Supervisor` on
  `SimTransport` drive one goal end-to-end → assert feedback (`planning`→`executing`)
  and terminal `SUCCESSFUL`. A second case: fake server aborts (`success=false`) →
  assert `PLANNING_FAILED` with the message relayed. Runs headless on abra; no GPU.
- **RT-safety:** unchanged — no new RT-path work; the core `RtSafety` gate still
  covers the sampler/pump against `SimTransport`.
- **Real-arm (attended, Milestone C):** the real cuRobo GPU node + our node on abra;
  a small/near base-frame target, slow, e-stop in hand, per `docs/on-robot-runbook.md`
  and the handoff §6 operational facts (`pkill -TERM -f kinova_arm_node`, best-effort
  `/joint_states` QoS, `ssh abra 'bash -lc "…"'`). Gated behind the sim integration.

## Milestones (each independently testable)

- **A — interfaces + core code:** `GoToEEPose.action` generates; `result_code`
  gains `kPlanningFailed`; both importable/linkable.
- **B — sim end-to-end with fake cuRobo (the proof, no GPU/hardware):** the two
  unit tests + the rclcpp integration test green on abra. Validates the whole pipe:
  server → cuRobo client → CommandSink → Supervisor → SimTransport → GoalRouter →
  result, plus the `PLANNING_FAILED` path.
- **C — attended real-arm run:** real cuRobo node + our node; a small pose goal
  succeeds on the physical arm. Gated behind B.

## Out of scope (future specs)

- `GoToJointConfig` (maps to cuRobo `plan_to_joints`) and `GoToPreset` (named-config
  registry) — next round.
- `speed_scale` / trajectory retiming, impedance execution + gains for `GoToEEPose`.
- cuRobo world management (`/set_world`), gripper actions, cuRobo's own
  `execute_trajectory` path (we deliberately execute via our Supervisor instead).
- Cross-tier arbitration (streaming vs actions) — hooks only (`sender_id`).
- TF transform of non-`base_link` target frames.
