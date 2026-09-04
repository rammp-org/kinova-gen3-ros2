# Guide: the planned-move actions

Three actions move the arm to a goal you describe, planning the collision-free
path for you rather than expecting one:

| action               | goal                                 | plans via               |
| -------------------- | ------------------------------------ | ----------------------- |
| `go_to_ee_pose`      | tool pose in `base_link`             | cuRobo `plan_to_pose`   |
| `go_to_joint_config` | explicit 7 joint angles (rad)        | cuRobo `plan_to_joints` |
| `go_to_preset`       | a **name** for a joint configuration | cuRobo `plan_to_joints` |

They contrast with `execute_joint_trajectory`, which takes an already-planned
trajectory. All three share one lifecycle — `PlannedMoveServer<ActionT>` — so
their Result and Feedback fields are identical and they behave the same on
cancel, rejection and failure. Only the goal differs.

`go_to_preset` is `go_to_joint_config` with a lookup in front: the name is
resolved to 7 joint angles and planned exactly the same way. **None of these is
a planner bypass** — nothing drives the arm straight at a target.

For the design rationale see
[`superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md`](superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md)
and the `GoToEEPose` guide's
[design spec](superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md).

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

The registry comes from two ROS parameters: `preset_names` lists the presets
and `presets.<name>` holds each one's 7 joint angles in radians. `home`
defaults to the cuRobo retract configuration, so a stock node always has one
usable preset.

```sh
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim \
  --ros-args -p preset_names:="['home','stow']" \
             -p presets.stow:="[0.0, 0.5, 3.0, -2.0, 0.0, 1.0, 1.5]"
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
node has no command sink, the pose frame is not `base_link`, `target_joints`
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

## Safety

The planned trajectory runs at **full planner speed**, and cuRobo plans to the
arm's real velocity limits. Read
[`on-robot-runbook.md`](on-robot-runbook.md) and keep the e-stop in hand before
the first real-arm run of any of these actions. Real-arm runs must pin the RT
loop to the isolated core (`--cpu 11`); `make sim` / `make real` do this.
