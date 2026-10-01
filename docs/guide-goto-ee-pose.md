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

It also carries the constraint fields, one move at a time: `--speed-scale S`,
`--hold none|level|fixed`. Bounds are checked client-side too,
because a rejection carries no payload — the reason only reaches the node's log.

### Demonstration scripts

Two companions live beside it in `test/`, both **dry run by default** and needing
an explicit `--go` to move anything:

| Script | What it is for |
| --- | --- |
| `sweep_constraints.py` | Runs one fixed motion under every constraint combination and tables the outcomes, re-homing between cases so the wall times compare. Each case carries an expectation, so a disagreement is flagged rather than left to the eye. |
| `send_goto_pose_tour.py` | A lap of large, widely-spaced waypoints, so a change in `speed_scale` or a held axis is visible across a room rather than needing a plot. |

`sweep_constraints.py` is the one to reach for when asking "does this constraint
actually do anything" — it includes cases that are *expected to be refused*,
which is how the start-must-match-the-goal rule shows itself.

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

`GoToEEPose` carries two optional fields. Both default to off, so a goal that
sets neither behaves exactly as before.

| field              | type               | default     | meaning                                       |
| ------------------ | ------------------ | ----------- | --------------------------------------------- |
| `speed_scale`      | `float64`          | `1.0`       | run the trajectory slower; `1.0` = as planned |
| `orientation_hold` | `OrientationHold`  | `HOLD_NONE` | keep the tool's orientation while it travels  |

**`speed_scale`** lowers the speed of the planned trajectory. The driver
executes it slower by dilating its executor clock; the plan itself is unchanged.
A value outside the driver's accepted range (its minimum up to `1.0`) or a
non-finite one is **refused, not clamped**.

**`orientation_hold`** has three modes and no frame:

| mode | effect |
| --- | --- |
| `HOLD_NONE` | the planner reorients freely |
| `HOLD_LEVEL` | roll and pitch held; spin about vertical stays free |
| `HOLD_FIXED` | the whole orientation held |

Four things callers meet as surprises:

- **LEVEL preserves tilt; it does not create level.** Roll and pitch are held at
  the *goal's* value, so a gripper that starts 20 degrees off stays 20 degrees
  off the whole way — faithfully, just not level. "Level" means "as level as you
  already are".
- **A goal orientation that disagrees with the current one is refused.** A hold
  keeps the orientation at the goal's value, so a goal that differs from where
  the arm is, on the held components, asks for two orientations at once. Send the
  current orientation as the goal's, or get there with an unconstrained move
  first. The planner refuses before planning, with the measured deviation.
- **A spoon needs FIXED, not LEVEL.** LEVEL leaves the spin about vertical free,
  which is right for anything symmetric about its upright axis — a cup, a bottle,
  a plate. A spoon is not symmetric: the bowl has to face a particular way, and
  LEVEL will plan happily while tipping the contents out.
- **"Level with the world" means "level with the robot base".** Those are the
  same thing while the arm is mounted level and stop being the same thing the
  moment it is not. Mount tilt belongs in the robot model, not in this field.

The hold is a planner *cost*, not a hard limit, so the planner measures the
trajectory it produced and refuses a plan whose worst deviation exceeds its
configured tolerance. A `SUCCESSFUL` result has been verified to hold within it.

The node translates the arm's `OrientationHold` into the planner's own type
(inside `CuroboPlanClient`), so callers never see the planner's message types.
An unknown mode is **refused**, never treated as `HOLD_NONE` — running an
unconstrained move for a caller who asked for a held one would look like
success.

**Refusals carry no payload.** `validate()` rejects a bad `speed_scale` or an
unknown hold mode, and the client receives a bare ROS action rejection with the
reason logged on the server only (tracked as `kinova-gen3-ros2#39`). A hold the
planner cannot satisfy is different: the goal is accepted and then settles
`PLANNING_FAILED (-7)` with the planner's reason — including the measured
deviation — in `error_string`, which the client does see.

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
