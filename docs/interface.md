# kinova-gen3-ros2 — Interface

**Rung:** ad-hoc (declared by this module, not standardized)
**Owner:** @SwapnilPande
**Tier:** experimental

## What it does

Drives a Kinova Gen3 7-DOF arm. It is the ROS 2 shell around
[`kinova-gen3-driver`](https://github.com/rammp-org/kinova-gen3-driver), which owns the
1 kHz real-time control loop; this module translates ROS messages to and from that
driver's plain value types and **never lets a ROS header reach the RT thread**.

It exposes four control tiers: trajectory execution, planned moves, arbitration
(who is allowed to command the arm), and setpoint streaming. Without it, nothing
in the system can move the arm.

**This module is two containers.** The `go_to_*` actions are clients of the cuRobo
planner's actions, so `deploy/compose.yaml` declares both the driver and
[`rammp-curobo`](https://github.com/rammp-org/RAMMP-CuRobo). Run the driver alone and
three of the four action servers accept goals that can never succeed.

## Publishes

| Topic             | Type                                   | Rate      | Meaning                                                                                 |
| ----------------- | -------------------------------------- | --------- | --------------------------------------------------------------------------------------- |
| `/joint_states`   | `sensor_msgs/JointState`               | ~100 Hz   | Seven arm joints plus `robotiq_85_left_knuckle_joint`. Best-effort QoS.                 |
| `/ee_state`       | `kinova_gen3_interfaces/EeState`       | ~100 Hz   | Tool pose and twist, `LOCAL_WORLD_ALIGNED`, from the same pump tick as `/joint_states`. |
| `/control_status` | `kinova_gen3_interfaces/ControlStatus` | on change | Who may command the arm: owner, `generation`, `estopped`, `rejected_count`. Latched.    |
| `/stream_status`  | `kinova_gen3_interfaces/StreamStatus`  | on change | The streaming session as core sees it. Latched.                                         |
| `/gripper_state`  | `kinova_gen3_interfaces/GripperState`  | 20 Hz     | `position`, `effort`, `current`, `present`.                                             |
| `/diagnostics`    | `diagnostic_msgs/DiagnosticArray`      | 1 Hz      | Three REP 107 tasks: `Arbitration`, `Arm`, `Gripper`.                                   |

`/joint_states` and `/ee_state` are best-effort, so CLI subscribers must match:
`ros2 topic echo --qos-reliability best_effort /joint_states`.

**Gripper `velocity` and `effort` are NaN in `/joint_states`, permanently.**
`sensor_msgs/JointState.effort` is documented in N·m or N; the gripper's is a 0..1
fraction of motor current, so it lives on `/gripper_state` where its units can be
stated. Core has no gripper velocity at all. NaN is the `sensor_msgs` convention for
"no measurement" — zero would be indistinguishable from "not moving".

**A sustained grasp reports a SMALL effort (~0.05).** Current spikes while the fingers
close and then settles to a low holding value. Anything keying off "high effort means
holding something" is backwards.

## Subscribes

| Topic                      | Type                           | Required | Meaning                                                       |
| -------------------------- | ------------------------------ | -------- | ------------------------------------------------------------- |
| `/estop`                   | `kinova_gen3_interfaces/EStop` | no       | Broadcast stop. Any node may publish. Volatile, deliberately. |
| `/setpoint/joint_position` | `JointSetpoint`                | no       | Joint angles, rad                                             |
| `/setpoint/joint_velocity` | `JointSetpoint`                | no       | Joint rates, rad/s                                            |
| `/setpoint/joint_torque`   | `JointSetpoint`                | no       | Joint torques, N·m                                            |
| `/setpoint/pose`           | `PoseSetpoint`                 | no       | Tool pose, base frame                                         |
| `/setpoint/twist`          | `TwistSetpoint`                | no       | Tool twist, base frame                                        |
| `/setpoint/wrench`         | `WrenchSetpoint`               | no       | No controller consumes this yet                               |
| `/setpoint/gripper`        | `GripperSetpoint`              | no       | `position`, `speed`, `force`                                  |

Setpoints are absolute and latest-wins, best-effort depth 1. A setpoint applies only
while a matching stream session is open and its token matches.

**`speed` and `force` are not sticky.** Core takes all three fields every call, so a
message carrying only `position` commands the other two to their defaults.

**`force` is a current ceiling, not a force setpoint.** The gripper closes at `speed`
toward `position` and stalls when it reaches the limit. There is no force servo on
this hardware by any path.

## Actions

The schema behind `rammp-alternative.yaml` has `publishes`/`subscribes` only, so this
section is the only place these are declared. They are the module's primary interface.

| Action                     | Type                     | Notes                                                 |
| -------------------------- | ------------------------ | ----------------------------------------------------- |
| `execute_joint_trajectory` | `ExecuteJointTrajectory` | Position or impedance; path and goal tolerance guards |
| `go_to_ee_pose`            | `GoToEEPose`             | **Requires the cuRobo planner**                       |
| `go_to_joint_config`       | `GoToJointConfig`        | **Requires the cuRobo planner**                       |
| `go_to_preset`             | `GoToPreset`             | **Requires the cuRobo planner**                       |

Under `arbitration_mode:=enforced`, every goal carries a token from
`/acquire_control`; an untokened goal is refused. A cancel carries no payload, so the
driver replays the goal's stored token.

## Services

| Service             | Type              | Meaning                                                                                        |
| ------------------- | ----------------- | ---------------------------------------------------------------------------------------------- |
| `/acquire_control`  | `AcquireControl`  | Mints a token. **Seizes**: succeeds even if another owner holds the arm, halting their motion. |
| `/release_control`  | `ReleaseControl`  | Refused unless the token matches the current owner                                             |
| `/revoke_control`   | `RevokeControl`   | Operator recovery; needs no token                                                              |
| `/open_stream`      | `OpenStream`      | Names a controller, returns the channels to publish on. `timeout_s` must be > 0.               |
| `/close_stream`     | `CloseStream`     | Ends the session                                                                               |
| `/list_controllers` | `ListControllers` | The registry with live availability — 7 of 8 available                                         |

## Parameters

| Name                    | Default    | Meaning                                                                                                                                                                        |
| ----------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `arbitration_mode`      | `disabled` | `enforced` or `disabled`. **Read-only at launch** — core's mode is a constructor argument with no setter, so a dynamic parameter would appear to work and silently do nothing. |
| `estop_clear_max_age_s` | `1.0`      | Age beyond which an e-stop *clear* is ignored. Engaging is never age-checked.                                                                                                  |
| `expect_gripper`        | `true`     | Whether to WARN on `/diagnostics` when no gripper is reported.                                                                                                                 |

## Requirements

Host networking and a shared IPC namespace (DDS shared-memory transport). `SYS_NICE`
plus `rtprio`/`memlock` ulimits for the RT loop — capabilities alone are not enough,
and `privileged` does **not** raise `RLIMIT_RTPRIO`. The planner needs
`runtime: nvidia`.

Real-time also needs the **host** tuned: `isolcpus`/`nohz_full`/`rcu_nocbs` on the RT
core. No container contract can express that, so a successful `ulimits` declaration
does not by itself mean you have real-time.
