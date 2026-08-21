# Guide: GoToEEPose

`GoToEEPose` moves the tool (`tool_frame`) to a target pose in `base_link`.
Unlike `ExecuteJointTrajectory`, which expects an already-planned trajectory,
`GoToEEPose` plans the collision-free path for you: the goal is delegated to
the external **cuRobo** node, and the returned trajectory is executed through
the same `Supervisor` that `ExecuteJointTrajectory` uses. Two ROS2 nodes are
involved:

- **`kinova_arm_node`** (this repo) — hosts the `go_to_ee_pose` action server,
  is a client of cuRobo's planning action, and drives the arm.
- **`rammp_curobo`** (external, `ChrissCox/RAMMP-CuRobo`) — plans a
  collision-free joint trajectory to the requested pose. It never moves the
  arm; it only plans.

For the full design rationale see
[`docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md`](superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md).

## Bring-up

Start the cuRobo planner alongside `kinova_arm_node`. Planning-only mode is
what we want — `execute:=true` is **not** needed since our node executes the
plan, not cuRobo:

```sh
ros2 launch rammp_curobo_ros planner.launch.py config:=gen3_real.yaml
```

Then start the arm node as usual (sim or real — see the top-level `README.md`
`Run` section). No extra flags are needed on `kinova_arm_node`: it already
publishes `/joint_states` at ~100 Hz, and cuRobo reads that topic to get the
live joint configuration it plans from. You do not need to supply a starting
configuration yourself.

## Calling it

Use the test client, which takes a target position and orientation in
`base_link` (quaternion in `xyzw` order):

```sh
python3 <ws>/src/kinova_arm_ros2/kinova_arm_ros2/test/send_goto_pose.py \
  --pos 0.45 0.10 0.35 --quat 0.0 0.0 0.0 1.0
```

Pick a pose **near the current tool pose** for a safe, local move — see
Safety below.

Client flags: `--pos X Y Z` (metres, `base_link`), `--quat X Y Z W`
(`base_link`, xyzw), `--sender-id` (arbitration hook, defaults to
`send_goto_pose`). The client prints feedback as it arrives and exits
non-zero if the terminal `error_code` isn't `0`.

## Result codes

| Code | Name | Meaning |
|---|---|---|
| `0` | `SUCCESSFUL` | Plan executed to completion. |
| `-1` | `INVALID_GOAL` | Goal rejected before planning — e.g. `target.header.frame_id` is not `base_link`. |
| `-4` | `PATH_TOLERANCE_VIOLATED` | Execution diverged from the planned trajectory beyond the guard. |
| `-6` | `PREEMPTED` | Goal was canceled (during planning or execution — see Cancelling below). |
| `-7` | `PLANNING_FAILED` | cuRobo returned no plan — unreachable/colliding target, the cuRobo action server is unavailable, or it's busy planning another goal (one plan at a time). `error_string` carries cuRobo's message. |

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

## Safety

**The planned trajectory executes at cuRobo's full planned speed — there is
no `speed_scale` or retiming in v1.** The returned `time_from_start` values
feed straight through to the Supervisor. This means the very first real-arm
`GoToEEPose` goal will move at whatever speed cuRobo's plan calls for, not a
conservative default.

Before running against the real arm:

- Pick a **small, near** target — do not send a target far from the current
  tool pose as your first real-arm goal.
- Stay **attended**, with the **e-stop in hand**.
- Follow the attended real-arm procedure in
  [`docs/on-robot-runbook.md`](on-robot-runbook.md) and log the run in its
  Runs section.
