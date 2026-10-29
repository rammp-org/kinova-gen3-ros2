# Guide: the planned-move actions

Three actions move the arm to a goal you describe, planning the collision-free
path for you rather than expecting one:

| action               | goal                                 | plans via               |
| -------------------- | ------------------------------------ | ----------------------- |
| `go_to_ee_pose`      | tool pose in `arm_mount`             | cuRobo `plan_to_pose`   |
| `go_to_joint_config` | explicit 7 joint angles (rad)        | cuRobo `plan_to_joints` |
| `go_to_preset`       | a **name** for a joint configuration | cuRobo `plan_to_joints` |

They contrast with `execute_joint_trajectory`, which takes an already-planned
trajectory. All three share one lifecycle — `PlannedMoveServer<ActionT>` — so
their Result and Feedback fields are identical and they behave the same on
cancel, rejection and failure. Only the goal differs.

`go_to_preset` is `go_to_joint_config` with a lookup in front: the name is
resolved to 7 joint angles and planned exactly the same way. **None of these is
a planner bypass** — nothing drives the arm straight at a target.

## Several targets in one motion

`go_to_ee_poses` and `go_to_joint_configs` are `go_to_ee_pose` and
`go_to_joint_config` through a list of targets. Each leg is planned as the
single-target action plans it, from where the leg before it ends, and the legs
are blended into one trajectory by cuRobo's `/rammp_curobo/blend`, so the arm
passes the targets between still moving instead of stopping at each.

- **Per leg:** `speed_scale`, and `orientation_hold` for poses. Empty means the
  default for every leg, one value is used for every leg, and otherwise there is
  one per target.
- **Per junction:** `blend_radius` (m), how far the tool may pass from that
  target to cut the corner. `0` stops on it; empty stops on every target. The
  last target always stops.
- **One for the whole motion:** `control_mode` and `gains`.

The planner cuts each corner only as far as the joint limits, collision, the
held legs' holds and the radius all allow, and stops on the target where nothing
can be cut. A leg that cannot be planned fails the goal before anything moves,
with `error_string` naming it (`leg 2/3: ...`; `blend: ...` if the blend itself
fails). Result and feedback are the single-target actions'; feedback's
`planner_state` carries the same prefix.

For the design rationale see
[`superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md`](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md)
and the `GoToEEPose` guide's
[design spec](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md).

## Bring-up

Two nodes: the external planner and this repo's arm node.

```sh
ros2 launch rammp_curobo_ros planner.launch.py config:=gen3_real.yaml
```

Then start `kinova_gen3_node` (sim or real — see the top-level `README.md`). No
extra flags: it is already a client of both cuRobo planning actions.

!!! warning "Both nodes must use the same RMW"
The planner container runs with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(see `ROS_FLAGS` in the `Makefile`). If `kinova_gen3_node` or your client
runs under the default FastRTPS, discovery partly succeeds — `ros2 action     list` shows the planner's actions — but goals are never answered, and the
action **hangs in `phase=planning` forever** rather than failing. A
`RTPS_READER_HISTORY ... payload size` error in the log is the tell. Export
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every shell involved; `make     sim` / `make real` already do.

## Sending a goal

`test/send_goto_joints.py` is a small client for the two joint-space actions.
It is **dry-run by default**; pass `--go` to actually execute.

```sh
# named configuration
python3 kinova_gen3_ros2/test/send_goto_joints.py --preset home --go

# explicit joints (rad)
python3 kinova_gen3_ros2/test/send_goto_joints.py \
    --joints 0 0.262 3.142 -2.269 0 0.96 1.571 --go

# relative to where the arm is now
python3 kinova_gen3_ros2/test/send_goto_joints.py --delta 0.15 --joint 6 --go
```

`test/send_goto_pose_sequence.py` does the same for `go_to_ee_pose`.

## Presets

Code against the preset **names**, never their joint angles. Every node has
these two:

| name   | meaning                                               |
| ------ | ----------------------------------------------------- |
| `home` | the working pose skills start from and return to      |
| `rest` | the arm parked out of the way when no task is running |

The angles behind each name belong to the installation: they depend on the
mount and on what surrounds the arm, so they live in the deployment's config
and change without a release. There is deliberately no `zero` preset: the
all-zeros configuration is not a safe place to send a mounted arm.

The registry comes from two ROS parameters: `preset_names` lists the presets
and `presets.<name>` holds each one's 7 joint angles in radians. `home` and
`rest` are always registered, even when `preset_names` leaves them out. Until
an installation overrides them, they use these defaults:

| name   | default (rad)                                      | Kinova pose |
| ------ | -------------------------------------------------- | ----------- |
| `home` | `[0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571]`    | Home        |
| `rest` | `[0.0, -0.349, 3.142, -2.356, 0.0, -0.873, 1.571]` | Retract\*   |

\* Joint 4 at -2.356 rad (-135 deg), not Retract's -2.548 (-146 deg). Factory
Retract self-collides in cuRobo and never plans.

Those defaults assume a level mount and an empty workspace, so the node logs a
warning at startup for each one still at its default, then a single
`presets:` line listing every registered name. A deployment overrides the
angles and may add presets beyond these two:

```sh
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim \
  --ros-args -p preset_names:="['home','rest','pour']" \
             -p presets.rest:="[0.0, -0.3, 3.142, -2.5, 0.0, -0.9, 1.571]" \
             -p presets.pour:="[0.0, 0.5, 3.0, -2.0, 0.0, 1.0, 1.5]"
```

An entry that is not exactly 7 values is **dropped with a warning** rather than
padded, and `go_to_preset` then rejects that name. An unknown name is rejected
outright — a preset never falls back to a default, because silently moving the
arm somewhere the caller did not ask for is worse than refusing.

## Results and feedback

Feedback arrives in two phases: `planning` (with cuRobo's `planner_state`
relayed in `planner_state`) then `executing` (with `fraction_complete` and the
live measured `actual`).

| `error_code`                 | meaning                                                  |
| ---------------------------- | -------------------------------------------------------- |
| `0` SUCCESSFUL               | planned and executed to the end of the trajectory        |
| `-1` INVALID_GOAL            | the supervisor refused the planned trajectory            |
| `-4` PATH_TOLERANCE_VIOLATED | the arm diverged from the planned path mid-execution     |
| `-6` PREEMPTED               | canceled, during planning or execution                   |
| `-7` PLANNING_FAILED         | cuRobo found no plan, or returned an empty/malformed one |

A goal is **rejected outright** (never accepted, so no result code) when the
node has no command sink, the pose frame is not `arm_mount`, `target_joints`
contains a non-finite value, or a preset name is unknown.

Cancel works in both phases: during planning it cancels the in-flight cuRobo
goal; during execution it goes through the supervisor. Either way the goal
settles exactly once, as `-6`.

### Expect `-4` for large moves in sim

Under `--sim` the arm does not actually follow commands — measured `q` stays
put — so the divergence guard (0.35 rad for these actions) fires on any move
larger than that. A small `--delta` returns `SUCCESSFUL`; `--preset home` from
the sim's zero configuration swings joint 3 through ~3.14 rad and correctly
returns `-4`. That is the guard working, not a failure. Use a real arm (or a
small delta) to exercise the success path.

## Speed and constraints per action

| action                     | `speed_scale` | `orientation_hold` |
| -------------------------- | :-----------: | :----------------: |
| `go_to_ee_pose`            |      yes      |        yes         |
| `go_to_joint_config`       |      yes      |         no         |
| `go_to_preset`             |      yes      |         no         |
| `go_to_ee_poses`           | yes, per leg  |    yes, per leg    |
| `go_to_joint_configs`      | yes, per leg  |         no         |
| `execute_joint_trajectory` |      yes      |         no         |

`speed_scale` (default `1.0`, range: the driver's minimum up to `1.0`) runs the
trajectory slower, by dilating the driver's executor clock. Out of range is
refused, not clamped.

**Joint-space goals carry speed only.** `GoToJointConfig` and `GoToPreset` plan
in joint space, where holding a tool orientation has no meaning, so they have no
`orientation_hold`. The absence is deliberate.

For `go_to_ee_pose`, both fields default to off. Three rules a caller will
otherwise meet as a confusing failure (full detail in the
[`GoToEEPose` guide](../guide-goto-ee-pose)):

- **`HOLD_LEVEL` preserves tilt, it does not create level**: a 45-degree tool
  stays at 45 the whole way.
- **A goal orientation disagreeing with the current one is refused**, not
  re-aimed — a hold keeps the orientation at the goal's value, so that asks for
  two orientations at once.
- **A spoon needs `HOLD_FIXED`**: `HOLD_LEVEL` leaves the spin about vertical
  free, which is fine for a cup and tips a spoon out.

**A refused goal tells the client nothing today.** Rejections are bare ROS
action rejections with no payload; the reason is logged server-side only
(tracked as `kinova-gen3-ros2#39`). This includes an out-of-range `speed_scale`.

## Safety

Unless `speed_scale` is set, the planned trajectory runs at **full planner
speed**, and cuRobo plans to the arm's real velocity limits. Set `speed_scale`
below 1 for a first real-arm run. Read
[`on-robot-runbook.md`](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/on-robot-runbook.md) and keep the e-stop in hand before
the first real-arm run of any of these actions. Real-arm runs must pin the RT
loop to the host's isolated core (`--cpu <n>`); `make sim` / `make real` do this
via `RT_CORE` (default 11).
