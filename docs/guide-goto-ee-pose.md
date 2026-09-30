# Guide: GoToEEPose

> `GoToEEPose` is one of three planned-move actions, alongside
> `GoToJointConfig` and `GoToPreset`, which share its lifecycle and its
> Result/Feedback exactly. See [`guide-goto-actions.md`](../guide-goto-actions)
> for the tier as a whole — including the RMW requirement that otherwise makes
> a goal hang in `planning` forever.

`GoToEEPose` moves the tool (`tool_frame`) to a target pose in `base_link`.
Unlike `ExecuteJointTrajectory`, which expects an already-planned trajectory,
`GoToEEPose` plans the collision-free path for you: the goal is delegated to
the external **cuRobo** node, and the returned trajectory is executed through
the same `Supervisor` that `ExecuteJointTrajectory` uses. Two ROS2 nodes are
involved:

- **`kinova_gen3_node`** (this repo) — hosts the `go_to_ee_pose` action server,
  is a client of cuRobo's planning action, and drives the arm.
- **`rammp_curobo`** (external, `ChrissCox/RAMMP-CuRobo`) — plans a
  collision-free joint trajectory to the requested pose. It never moves the
  arm; it only plans.

For the full design rationale see
[`docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md`](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md).

## Bring-up

Start the cuRobo planner alongside `kinova_gen3_node`. Planning-only mode is
what we want — `execute:=true` is **not** needed since our node executes the
plan, not cuRobo:

```sh
ros2 launch rammp_curobo_ros planner.launch.py config:=gen3_real.yaml
```

Then start the arm node as usual (sim or real — see the top-level `README.md`
`Run` section). No extra flags are needed on `kinova_gen3_node`, and you do not
supply a starting configuration yourself: the arm node reads its own measured
joint state and states it in the plan request, so cuRobo plans from exactly the
configuration the arm is standing in.

That is deliberate. cuRobo *can* source the start state itself, by subscribing
to `/joint_states`, if the request leaves `start_joints` empty — but then the
planner depends on the robot being present and on the two nodes' QoS matching,
and it will accept a joint state up to 2 s old. Planning from a configuration
the arm has already left, while the arm node executes from the live one, is a
real divergence. We never send the empty form.

## Calling it

Use the test client, which takes a target position and orientation in
`base_link` (quaternion in `xyzw` order):

```sh
python3 <ws>/src/kinova_gen3_ros2/kinova_gen3_ros2/test/send_goto_pose.py \
  --pos 0.45 0.10 0.35 --quat 0.0 0.0 0.0 1.0
```

Pick a pose **near the current tool pose** for a safe, local move — see
Safety below.

Client flags: `--pos X Y Z` (metres, `base_link`), `--quat X Y Z W`
(`base_link`, xyzw), `--sender-id` (arbitration hook, defaults to
`send_goto_pose`). The client prints feedback as it arrives and exits
non-zero if the terminal `error_code` isn't `0`.

## Result codes

| Code | Name                      | Meaning                                                                                                                                                                                            |
| ---- | ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | `SUCCESSFUL`              | Plan executed to completion.                                                                                                                                                                       |
| `-1` | `INVALID_GOAL`            | Goal rejected before planning — e.g. `target.header.frame_id` is not `base_link`.                                                                                                                  |
| `-4` | `PATH_TOLERANCE_VIOLATED` | Execution diverged from the planned trajectory beyond the guard.                                                                                                                                   |
| `-6` | `PREEMPTED`               | Goal was canceled (during planning or execution — see Cancelling below).                                                                                                                           |
| `-7` | `PLANNING_FAILED`         | cuRobo returned no plan — unreachable/colliding target, the cuRobo action server is unavailable, or it's busy planning another goal (one plan at a time). `error_string` carries cuRobo's message. |

`error_code = -5` (`GOAL_TOLERANCE_VIOLATED`) does not apply to this action.

## Feedback

Feedback has two phases, reflected in the `phase` field:

- **`planning`** — `planner_state` relays cuRobo's own feedback `state`
  string; `fraction_complete` stays `0`.
- **`executing`** — the plan is running through the Supervisor;
  `fraction_complete` tracks execution progress and `actual` carries the live
  measured joint position, same as `ExecuteJointTrajectory` feedback.

## Cancelling

Cancel behavior depends on which phase the goal is in when the cancel
request arrives:

- **Canceling while planning** settles the goal `PREEMPTED` immediately and
  the arm never moves — the in-flight cuRobo plan request is canceled and any
  plan that races ahead and succeeds is discarded.
- **Canceling while executing** cancels the in-flight trajectory through the
  Supervisor (the same path `ExecuteJointTrajectory` cancellation uses),
  which settles the goal `PREEMPTED` once the arm has stopped.

## Speed, locks and approach

`GoToEEPose` carries three optional fields. All default to off, so a goal that
sets none of them behaves exactly as before.

| field             | type             | default | meaning                                       |
| ----------------- | ---------------- | ------- | --------------------------------------------- |
| `speed_scale`     | `float64`        | `1.0`   | run the trajectory slower; `1.0` = as planned |
| `axis_lock`       | `ToolAxisLock`   | none    | hold tool-pose components fixed throughout    |
| `approach_offset` | `ApproachOffset` | `0.0`   | arrive along one axis, from a set distance    |

**`speed_scale`** lowers the speed of the planned trajectory. The driver
executes it slower by dilating its executor clock; the plan itself is unchanged.
A value outside the driver's accepted range (its minimum up to `1.0`) or a
non-finite one is **refused, not clamped**.

**`axis_lock`** (`lock_roll`, `lock_pitch`, `lock_yaw`, `lock_x`, `lock_y`,
`lock_z`, plus a `reference_frame`) holds those components fixed for the whole
trajectory. Two things callers meet as surprises:

- **"Locked" means unchanged, not level.** A locked component is held at the
  *goal's* value, so the start pose must already match the goal on it. A tool at
  45 degrees planning to a 45-degree goal stays at 45 the whole way. If the start
  does not match, the planner refuses the goal.
- **An approach is not independent of locking.** `approach_offset` (`distance`,
  `axis`, `at_fraction`) makes the path pass near a point `distance` back from the
  goal along one axis. The planner does this by holding the **other five** pose
  components at the goal's values while travelling along the freed axis. Asking
  for an approach therefore also asks for those five locks, and the start pose
  must already match the goal on them, even if `axis_lock` is all false. The
  approach is a preference, not a waypoint: the path passes near the offset
  without stopping at it. `axis_lock.reference_frame` governs both the lock and
  the approach axis.

The node translates the arm's lock and approach into the planner's own types
(inside `CuroboPlanClient`), so callers never see the planner's message types.

**A refused goal tells the client nothing today.** When `validate()` rejects a
goal (bad `speed_scale`, negative approach `distance`, `at_fraction` outside
(0, 1), and so on) the client receives a bare ROS action rejection with no
payload; the reason is logged on the server only. Check the node log when a goal
is rejected. Returning the reason to the client is tracked as
`kinova-gen3-ros2#39`.

## Safety

**Unless you set `speed_scale`, the trajectory runs at cuRobo's full planned
speed.** The returned `time_from_start` values feed straight through to the
Supervisor, so a goal that leaves `speed_scale` at its default of `1.0` moves
at whatever speed cuRobo's plan calls for, not a conservative one. For a first
real-arm goal, set `speed_scale` well below 1 (see
[Speed, locks and approach](#speed-locks-and-approach)).

Before running against the real arm:

- Pick a **small, near** target — do not send a target far from the current
  tool pose as your first real-arm goal.
- Stay **attended**, with the **e-stop in hand**.
- Follow the attended real-arm procedure in
  [`docs/on-robot-runbook.md`](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/on-robot-runbook.md) and log the run in its
  Runs section.
