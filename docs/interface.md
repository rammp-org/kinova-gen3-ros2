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

**This module is three containers.** The driver reads the arm's model from
`/arm/robot_description`, which `arm_state_publisher` (this image's
`kinova_gen3_description`) renders from the robot's `mount.yaml`; without it the driver
exits after 30 s. The `go_to_*` actions are clients of the cuRobo planner's actions, so
`deploy/compose.yaml` declares the driver, `arm_state_publisher` and
[`rammp-curobo`](https://github.com/rammp-org/RAMMP-CuRobo). Run the driver without the
planner and three of the four action servers accept goals that can never succeed.

## Publishes

| Topic                       | Type                                    | Rate      | Meaning                                                                                                                                                                                                                                                                                                                              |
| --------------------------- | --------------------------------------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `/joint_states`             | `sensor_msgs/JointState`                | ~50 Hz    | Seven arm joints (`arm_joint_1`…`arm_joint_7`, named by the model) plus `arm_robotiq_85_left_knuckle_joint`. Best-effort QoS. Pump ticks decimated by `state_publish_divisor` (default 2).                                                                                                                                           |
| `/ee_state`                 | `rammp_arm_interfaces/EeState`          | ~50 Hz    | Pose of `--ee-frame` (default `arm_end_effector_link`, the flange; `arm_tool_frame` is 0.120 m beyond it) in `arm_mount`, twist and measured external wrench `LOCAL_WORLD_ALIGNED`, from the same pump tick as `/joint_states`. Which estimate the wrench is: see [The `/ee_state` wrench](#the-ee_state-wrench).                    |
| `/control_status`           | `rammp_common_interfaces/ControlStatus` | on change | Who may command the arm: owner, `generation`, `estopped`, `rejected_count`. Latched.                                                                                                                                                                                                                                                 |
| `/stream_status`            | `rammp_arm_interfaces/StreamStatus`     | on change | The streaming session as core sees it. Latched.                                                                                                                                                                                                                                                                                      |
| `/gripper_state`            | `rammp_arm_interfaces/GripperState`     | 20 Hz     | `position`, `effort`, `current`, `present`.                                                                                                                                                                                                                                                                                          |
| `/contact_monitor/residual` | `sensor_msgs/JointState`                | 50 Hz     | Only with `contact_debug_topic:=true` (off by default), for tuning: the contact observer's `r` (estimated external joint torque, N·m) in `effort`, `name` the seven arm joints as on `/joint_states`, `position` and `velocity` empty. Reliable QoS. Not created with `contact_detection:=off`. See [Contact stops](#contact-stops). |
| `/diagnostics`              | `diagnostic_msgs/DiagnosticArray`       | 1 Hz      | Five REP 107 tasks: `Arbitration`, `Arm`, `Arm link`, `Gripper`, `Contact detection`.                                                                                                                                                                                                                                                |

`/joint_states` and `/ee_state` are best-effort, so CLI subscribers
must match: `ros2 topic echo --qos-reliability best_effort /joint_states`.

**Gripper `velocity` and `effort` are NaN in `/joint_states`, permanently.**
`sensor_msgs/JointState.effort` is documented in N·m or N; the gripper's is a 0..1
fraction of motor current, so it lives on `/gripper_state` where its units can be
stated. Core has no gripper velocity at all. NaN is the `sensor_msgs` convention for
"no measurement" — zero would be indistinguishable from "not moving".

**A sustained grasp reports a SMALL effort (~0.05).** Current spikes while the fingers
close and then settles to a low holding value. Anything keying off "high effort means
holding something" is backwards.

## Subscribes

| Topic                      | Type                            | Required | Meaning                                                                                                                                                                                       |
| -------------------------- | ------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/arm/robot_description`   | `std_msgs/String`               | **yes**  | The arm's model (URDF), latched, from `arm_state_publisher`. Read once at startup (30 s timeout, then exit 1); a different model published later logs an ERROR until the driver is restarted. |
| `/estop`                   | `rammp_common_interfaces/EStop` | no       | Broadcast stop. Any node may publish. Volatile, deliberately.                                                                                                                                 |
| `/setpoint/joint_position` | `JointSetpoint`                 | no       | Joint angles, rad                                                                                                                                                                             |
| `/setpoint/joint_velocity` | `JointSetpoint`                 | no       | Joint rates, rad/s                                                                                                                                                                            |
| `/setpoint/joint_torque`   | `JointSetpoint`                 | no       | Joint torques, N·m                                                                                                                                                                            |
| `/setpoint/pose`           | `PoseSetpoint`                  | no       | Tool pose, `arm_mount`                                                                                                                                                                        |
| `/setpoint/twist`          | `TwistSetpoint`                 | no       | Tool twist, `arm_mount` axes                                                                                                                                                                  |
| `/setpoint/gripper`        | `GripperSetpoint`               | no       | `position`, `speed`, `force`                                                                                                                                                                  |

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

A goal stopped by an unexpected contact settles `-11` CONTACT_STOP (`halted: unexpected contact`), distinct from `-9` HALTED, because it needs a `/clear_faults` before the arm
moves again. While a contact stop is latched, new goals are rejected outright and the
node log says why (`rejecting goal: stopped on unexpected contact (joint N); call /clear_faults`). See [Contact stops](#contact-stops).

An `execute_joint_trajectory` goal must name its joints `arm_joint_1`…`arm_joint_7`
in `trajectory.joint_names`, in that order (the names `/joint_states` carries), and
a non-empty `path_tolerance` or `goal_tolerance` must name them the same way.
Positions are mapped by index, so any other list, an empty one included, is refused.

Under `arbitration_mode:=enforced`, every goal carries a token from
`/acquire_control`; an untokened goal is refused. A cancel carries no payload, so the
driver replays the goal's stored token.

## Services

| Service                   | Type                   | Meaning                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ------------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `/acquire_control`        | `AcquireControl`       | Mints a token, or adopts the caller's non-zero `token`. **Seizes**: succeeds even if another owner holds the arm, halting their motion. Re-acquiring with the same key halts too.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `/release_control`        | `ReleaseControl`       | Refused unless the token matches the current owner                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `/revoke_control`         | `RevokeControl`        | Operator recovery; needs no token                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `/open_stream`            | `OpenStream`           | Names a controller, returns the channels to publish on. `timeout_s` must be > 0. Refused with `error_code` −11 CONTACT_STOP while a contact stop is latched (an arm that is not ready answers −10 first). A stream open at a contact stop is closed (core records the close cause as contact, not halted); `/stream_status` carries no cause, so it shows as `open=false` with `Contact detection` at ERROR on `/diagnostics`.                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `/close_stream`           | `CloseStream`          | Ends the session                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `/list_controllers`       | `ListControllers`      | The registry with live availability — 7 of 8 available                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `/clear_faults`           | `std_srvs/Trigger`     | Clears whatever is latched: a contact stop first, then a latched arm fault, taking the arm back into low-level servoing. One call clears both. With only a contact stop (link `running`): `success`, `contact stop cleared`, and the arm holds where it is. With a contact stop and the link `connecting`, the same reply comes back before the latch drops: it drops when the RT loop next runs (once the arm is reached), and `Contact detection` stays ERROR until then. With a fault: answers within ~10 s, `success` only if the driver is commanding again; `message` is the resulting state and why (prefixed `contact stop cleared; ` if one was cleared too). With no outcome in 10 s it says so; the clear may still finish later (watch `Arm link`). `stopping` (at once) while the node shuts down. Refused (`not faulted (<state>)`) when nothing is latched. Needs no token. |
| `/set_contact_thresholds` | `SetContactThresholds` | Switches the contact thresholds: `PROFILE_SENSITIVE`, `PROFILE_DEFAULT`, `PROFILE_LOOSE` (from the parameters below, `custom` empty), or `PROFILE_CUSTOM` with exactly 7 values in `custom` (N·m, joints 1..7). Holds until the next call. **Token-gated** like a goal. Refused, with `message` naming the problem and the joint and limit, for: an unknown profile, `custom` of the wrong length, a value that is not finite, ≤ 0 or above `contact_thresholds_ceiling`, or no valid token. Nothing is clamped. `message` is empty when accepted. `profile` and `thresholds` in the response are what is active after the call, accepted or not. Refused with `contact detection not available` when `contact_detection:=off`; nothing is active then, so the response carries `profile` 0 (`PROFILE_DEFAULT`) and all-zero `thresholds`.                                                 |

### The arm link

The node does not exit on arm-side events. The `Arm link` diagnostics task
says which state the link is in:

| State        | `Arm link` level                               | Meaning                                                                                                                                                                                                                                                     | What happens to goals and streams                                                                                                                      |
| ------------ | ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `running`    | OK                                             | Low-level servoing, the driver is commanding                                                                                                                                                                                                                | accepted                                                                                                                                               |
| `connecting` | STALE: `waiting for the arm at <ip>: <reason>` | The arm is off or booting, the link is down, or it is renegotiating. Retried every second, forever. A link lost while running goes to `faulted` once the arm is back.                                                                                       | in-flight goals settle `-9` HALTED (`halted: arm fault or connection lost`); new ones are refused; `/joint_states` stops (no stale state is published) |
| `faulted`    | ERROR: the fault                               | E-stop, protective stop, the arm left low-level (web app), a command the arm rejected, or the link was lost while running (`connection lost: …`). The arm is handed back to its base, so the Kinova controller and web app work. Waits for `/clear_faults`. | same as above, except `/joint_states` keeps publishing (feedback keeps flowing while the base owns the arm)                                            |

After a clear the arm holds where it is now. It never resumes an
interrupted goal. The `Arm` task goes STALE when feedback is more than 1 s old.

### Contact stops

The driver watches a momentum observer's estimate `r` of the external torque on each
joint (N·m). Any joint with `|r_i|` above its threshold `θ_i` for
`contact_debounce_cycles` consecutive RT cycles (default 5, 5 ms at 1 kHz) is a
contact. Detection runs whenever the RT loop runs, including the
idle hold between goals: touching a still arm trips it like touching a moving one.
After each start, and after each clear, `r` settles for `5/contact_gain` s (0.2 s)
before anything counts (`phase=SETTLING`). Nothing is monitored while the RT loop is
not running (the link is not `running`, see [The arm link](#the-arm-link)):
`Contact detection` then shows the state from the end of the last
run, `armed` included, and before the first run they read `settling`.

What happens on a trip, with `contact_detection:=enforce`:

1. **Float.** The arm switches to a float mode in the RT loop:
   `τ = g(q) − D q̇ − K_s (q − q_a)`. It yields to the push, slows under the joint
   damping `D`, and stays where it is left; the weak spring `K_s` holds it against
   model creep with at most ~1 N·m (j1–j4) or ~0.3 N·m (j5–j7). From a position-mode
   move, float is adopted at the next cycle boundary.
1. **Latch.** It stays in float until a `/clear_faults`. In-flight goals settle `-11`
   CONTACT_STOP; new goals, stream opens and gripper setpoints are refused with
   `stopped on unexpected contact; call /clear_faults`. As on any halt, the gripper
   stops being commanded and keeps its grip; it is never commanded open. The latch survives everything except a clear: arm faults, link
   loss and reconnect, e-stop and ownership changes. If the arm comes back from a link
   outage still latched, it comes back in float, not the position hold.
1. **Report.** `Contact detection` on `/diagnostics` goes ERROR:
   `stopped on contact: joint N, |r| = x N·m > θ = y N·m at the trip (profile now P); call /clear_faults`.
   `θ` is the threshold at the trip; `P` is the profile now, which may have changed since.
   Its key-values carry `phase` `latched` and the trip record: `last_trip_joint`
   (1-based), and `last_trip_residual_nm`, `last_trip_thresholds_nm`,
   `last_trip_q_rad`, `last_trip_qd_rad_s` at the trip.
1. **Clear.** `/clear_faults` clears it; the arm holds where it is now in position mode
   and never resumes the stopped goal. If someone is still pressing on the arm, it
   trips again once `r` has settled (0.2 s), which is safe.

| `Contact detection` level | Message                                                                         |
| ------------------------- | ------------------------------------------------------------------------------- |
| OK                        | `armed (profile P)`                                                             |
| WARN                      | `settling` (for 0.2 s after a start or a clear, and until the first run)        |
| WARN                      | `observe mode: not stopping the arm; N would-be trips[, last on joint J]`       |
| WARN                      | `DISABLED (contact_detection=off)`                                              |
| ERROR                     | `stopped on contact: joint N, \|r\| = … > θ = … at the trip (profile now P); …` |

**Watching `r`.** `Contact detection` (1 Hz) carries `detection`, `phase`, `profile`,
`residual_nm`, `thresholds_nm`, `would_trips`, `bad_samples` and, after the first trip
or would-be trip, the trip record above; a clear does not reset it. For tuning,
`contact_debug_topic:=true` publishes `r` at 50 Hz on `/contact_monitor/residual`
(`sensor_msgs/JointState`, `effort` = `r` in N·m), which PlotJuggler or Foxglove plots
next to `/joint_states`.

**Profiles.** The driver starts in `default`. A client expecting contact switches to a
looser profile for that motion and back afterwards: the bite transfer, carrying a
payload heavy enough to show in `r` (a filled container), scooping, pressing a utensil
onto a plate. `PROFILE_CUSTOM` is how thresholds are tuned live on the arm. The named
profiles and the ceiling are startup parameters and change only with a restart: a ROS
parameter set carries no token, so a live edit would let any process retune the arm.

**`observe` and `off`.** `contact_detection:=observe` runs the detector and reports
would-be trips (counted in the `Contact detection` summary and `would_trips`, `phase`
stays `armed`) but never
stops the arm; it is how false-positive runs are done safely. `contact_detection:=off`
builds no detector at all: `Contact detection` reads `DISABLED`,
`/set_contact_thresholds` is refused, and nothing stops the arm on contact. Both are
deliberate choices made at launch, not switchable at runtime, and both print a WARN at
startup and keep the `Contact detection` task at WARN for as long as the node runs.

**Blind spot.** A force along a line through the joint axes (a near-extended arm pushed
along its length) makes almost no joint torque and is not detected.

### False positives

Peak `|r|` per joint with nobody touching the arm, measured on the arm in `observe` mode.
The monitor runs at 1 kHz, but these peaks are the largest of the 50 Hz samples the
bench recorded, so the in-loop peak runs higher: on the speed-1.0 position tour the 1 kHz trip
record caught j1 at 8.01 N·m, against 5.89 in the samples. Read the margin to `θ` with
that in mind.

| peak `\|r\|`, N·m               | j1   | j2   | j3   | j4   | j5   | j6   | j7   |
| ------------------------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| position, speed 0.6, 2026-10-29 | 4.57 | 2.69 | 3.44 | 1.55 | 1.27 | 1.00 | 0.19 |
| position, speed 0.6, 2026-10-30 | 2.9  | 2.6  | 1.6  | 1.4  | 0.6  | 1.2  | 0.3  |
| position, speed 1.0             | 5.89 | 3.49 | 4.40 | 2.17 | 1.08 | 1.12 | 0.19 |
| impedance, speed 0.6            | 1.97 | 3.48 | 0.63 | 1.65 | 0.70 | 0.94 | 0.27 |
| impedance, speed 1.0            | 2.31 | 1.82 | 0.93 | 1.48 | 0.67 | 0.89 | 0.26 |
| default `θ`                     | 16   | 16   | 12   | 12   | 4    | 4    | 4    |

- **Still, the floor is the sensors:** σ ≤ 0.01 N·m on every joint. The force at the
  gripper that trips at home is about 23–48 N at the default profile, depending on
  direction, and 11–25 N at `PROFILE_SENSITIVE`.
- **Moving, the floor is model error.** In position mode `r` grows with acceleration
  (most likely the model's rotational inertia): about 3 N·m on j1 at speed 0.6 and
  about 6 N·m at 1.0. The 1.0 tour gave one would-be trip at the earlier default of
  `[8, 8, 6, 6, 2, 2, 2]` (j1 8.01 > 8), which is why that is now `sensitive` and the
  default is doubled. Impedance mode stays under 3.5 N·m on every joint.
- **Payloads read as external torque.** A held object's weight shows in `r` as a
  steady offset. Switch to `PROFILE_LOOSE` around grasps and while carrying anything
  heavier than a utensil.
- **The position-to-float switch.** A trip from position mode (a move, or the idle
  hold) switches the actuators to torque mode, and today that switch gives them zero
  torque for about 160 ms, so the arm can droop before float takes over
  ([driver#92](https://github.com/rammp-org/kinova-gen3-driver/issues/92), fixed by
  driver PR #94, not merged yet). A trip from an impedance move is already in torque
  mode.

The full bench report, with the tap tests, is the driver's
[contact detection bench report](https://github.com/rammp-org/kinova-gen3-driver/blob/dev/docs/integration/contact_detection_bench.md).
Open work: a fast channel to tell taps from slow model error and payloads
([driver#111](https://github.com/rammp-org/kinova-gen3-driver/issues/111)).

### The `/ee_state` wrench

The wrench on `/ee_state` is what the environment applies to the tool, mapped from
an external joint torque estimate through the Jacobian. Which estimate it is depends
on the contact monitor; the message does not say
([driver#112](https://github.com/rammp-org/kinova-gen3-driver/issues/112)):

| Estimate     | When                                                                                                                                                                          | Character                                                                                                                                                                    |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Observer     | `contact_detection:=enforce` or `observe`, once the observer has settled, latched or not                                                                                      | The contact observer's `r`. Models the arm's own inertia, so it stays accurate in motion. Measured about 25 ms behind on hand taps (first-order, time constant 1/K = 40 ms). |
| Quasi-static | `contact_detection:=off`; the ~200 ms the observer settles after each start, fault recovery or reconnect, and contact clear; and while the arm link waits for `/clear_faults` | `g(q) − τ`. Exact at rest with no lag, but the arm's own acceleration reads as force while it moves.                                                                         |

On a position-mode tour at speed 0.6 with nobody touching, the force error at the
flange was median 3.3 / p95 8.0 / max 13.7 N quasi-static, and 1.9 / 5.4 / 6.2 N from
the observer. The lag is fine for monitoring and choosing thresholds at 50 Hz; a client
doing fast force control that wants the zero-lag quasi-static estimate computes
`g(q) − effort` from `/joint_states` itself. Both
read a held payload as external force
([driver#111](https://github.com/rammp-org/kinova-gen3-driver/issues/111)). NaN while
the arm reports a fault.

## Parameters

| Name                           | Default                                   | Meaning                                                                                                                                                                        |
| ------------------------------ | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `arbitration_mode`             | `disabled`                                | `enforced` or `disabled`. **Read-only at launch** — core's mode is a constructor argument with no setter, so a dynamic parameter would appear to work and silently do nothing. |
| `estop_clear_max_age_s`        | `1.0`                                     | Age beyond which an e-stop *clear* is ignored. Engaging is never age-checked.                                                                                                  |
| `expect_gripper`               | `true`                                    | Whether to WARN on `/diagnostics` when no gripper is reported.                                                                                                                 |
| `contact_detection`            | `enforce`                                 | `enforce`, `observe` or `off`. See [Contact stops](#contact-stops). **Read-only at launch**, like every `contact_*` and `float_*` parameter.                                   |
| `contact_gain`                 | `25.0`                                    | Observer gain `K`, rad/s. Settling after a start or clear is `5/K` s.                                                                                                          |
| `contact_debounce_cycles`      | `5`                                       | Consecutive RT cycles over `θ` that make a trip.                                                                                                                               |
| `contact_thresholds_sensitive` | `[8.0, 8.0, 6.0, 6.0, 2.0, 2.0, 2.0]`     | `PROFILE_SENSITIVE`, N·m, joints 1..7.                                                                                                                                         |
| `contact_thresholds_default`   | `[16.0, 16.0, 12.0, 12.0, 4.0, 4.0, 4.0]` | `PROFILE_DEFAULT`, active at start.                                                                                                                                            |
| `contact_thresholds_loose`     | `[24.0, 24.0, 18.0, 18.0, 6.0, 6.0, 6.0]` | `PROFILE_LOOSE`.                                                                                                                                                               |
| `contact_thresholds_ceiling`   | `[30.0, 30.0, 22.0, 22.0, 7.0, 7.0, 7.0]` | Per-joint upper bound on every profile and every custom value.                                                                                                                 |
| `float_damping`                | `[5.0, 5.0, 5.0, 5.0, 1.5, 1.5, 1.5]`     | Float-mode joint damping `D`, N·m·s/rad.                                                                                                                                       |
| `float_spring`                 | `[28.6, 28.6, 28.6, 28.6, 8.6, 8.6, 8.6]` | Float-mode anchor spring `K_s`, N·m/rad.                                                                                                                                       |
| `float_drag_band_deg`          | `2.0`                                     | How far the float anchor trails the arm, deg.                                                                                                                                  |
| `contact_debug_topic`          | `false`                                   | Publish `r` on `/contact_monitor/residual` at 50 Hz, for tuning. Ignored with `contact_detection:=off`.                                                                        |

The thresholds come from the 2026-10-30 push tests (see
[False positives](#false-positives)) and are still being tuned on the arm. Every array must have
exactly 7 entries, and every threshold must be finite, > 0 and ≤ the ceiling; anything
else is fatal at startup (the node logs which parameter and joint and exits 1). Write
array entries as floats (`8.0`, not `8`): a YAML integer array is a type error.

## Requirements

Host networking and a shared IPC namespace (DDS shared-memory transport). `SYS_NICE`
plus `rtprio`/`memlock` ulimits for the RT loop — capabilities alone are not enough,
and `privileged` does **not** raise `RLIMIT_RTPRIO`. The planner needs
`runtime: nvidia`.

Real-time also needs the **host** tuned: `isolcpus`/`nohz_full`/`rcu_nocbs` on the RT
core. No container contract can express that, so a successful `ulimits` declaration
does not by itself mean you have real-time.
