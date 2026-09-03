# kinova_gen3_ros2

ROS2 Humble frontend for the Kinova Gen3 low-level driver
([`rammp-org/kinova-gen3-driver`](https://github.com/rammp-org/kinova-gen3-driver), the "core").

The core is ROS-free plain CMake: it owns the 1 kHz real-time control loop, the
control modes, and the `Supervisor` that arbitrates goals. This repo is the thin
ROS2 shell around it — it advertises an action server, streams joint state, and
translates ROS2 messages to/from the core's plain value types. **No ROS2 header
ever reaches the RT thread.**

Design docs: `docs/superpowers/specs/2026-08-12-ros2-backend-realization-design.md`
(architecture) and `docs/on-robot-runbook.md` (attended real-arm procedure).

## Packages

| Package | Type | Contents |
|---|---|---|
| `kinova_gen3_interfaces` | `ament_cmake` + `rosidl` | `ExecuteJointTrajectory.action`, `JointImpedanceGains.msg`. Interface definitions only. |
| `kinova_gen3_ros2` | `ament_cmake` | `message_mapping` + `ros2_backend` libraries and the `kinova_gen3_node` executable. |

```
kinova_gen3_interfaces/
  action/ExecuteJointTrajectory.action
  msg/JointImpedanceGains.msg
kinova_gen3_ros2/
  include/kinova_gen3_ros2/{ros2_backend,message_mapping}.h
  src/message_mapping.cpp     ROS2 msg <-> kinova::interface value types (no rclcpp)
  src/ros2_backend.cpp        the ONLY unit that includes rclcpp/rclcpp_action
  src/bringup_node.cpp        DI wiring + main()
  test/message_mapping_test.cpp   gtest (runs under `colcon test`)
  test/send_trajectory.py         rclpy action client used as the integration harness
kinova_gen3.repos            vcs source list vendoring the core into the colcon ws
scripts/                    build + e2e helper scripts
docs/                       design spec, on-robot runbook
```

## Architecture

The core defines three ports (`kinova_lowlevel/interface/ports.h`);
`Ros2Backend` sits on both sides of them:

- **`ActionServerPort`** (implemented by `Ros2Backend`) — the Supervisor's
  *sampler* thread calls `publish_feedback()` / `settle()` to push feedback and
  terminal results out to the ROS2 goal handle.
- **`StreamPort`** (implemented by `Ros2Backend`) — the Supervisor's *pump*
  thread calls `publish_state()` (~100 Hz) to publish `/joint_states`.
- **`CommandSink`** (implemented by the Supervisor) — the backend's rclcpp
  callbacks call `on_trajectory_goal()` / `on_trajectory_accepted()` /
  `on_trajectory_cancel()` on inbound goals.

Threading in `kinova_gen3_node`: the RT executor runs the 1 kHz loop on the **main
thread**; the rclcpp `SingleThreadedExecutor` and the telemetry-ring drain each
get their own thread. Everything ROS2 is non-RT and reaches the loop only through
the Supervisor's existing lock-free seams. `SIGINT` and `SIGTERM` both set the
stop flag, which unblocks the RT loop and triggers `sup.stop()` +
`transport.safe_shutdown()`.

## ROS2 interface

Node name: **`kinova_gen3_node`**.

### Action servers

| Name | Type |
|---|---|
| `execute_joint_trajectory` | `kinova_gen3_interfaces/action/ExecuteJointTrajectory` |
| `go_to_ee_pose` | `kinova_gen3_interfaces/action/GoToEEPose` |
| `go_to_joint_config` | `kinova_gen3_interfaces/action/GoToJointConfig` |
| `go_to_preset` | `kinova_gen3_interfaces/action/GoToPreset` |

Goal:

```
trajectory_msgs/JointTrajectory  trajectory
control_msgs/JointTolerance[]    path_tolerance      # empty => guard disabled
control_msgs/JointTolerance[]    goal_tolerance      # empty => guard disabled
builtin_interfaces/Duration      goal_time_tolerance
uint8   control_mode             # 0 = POSITION, 1 = IMPEDANCE
uint8   preemption               # 0 = QUEUE,    1 = LATEST_WINS
JointImpedanceGains gains        # kq[7], zeta, torque_limit[7]; used iff IMPEDANCE
string  sender_id
```

Result: `error_code` (`SUCCESSFUL=0`, `INVALID_GOAL=-1`, `PATH_TOLERANCE_VIOLATED=-4`,
`GOAL_TOLERANCE_VIOLATED=-5`, `PREEMPTED=-6`), `error_string`, `final_error`.

Feedback: `desired` / `actual` / `error` (`actual` is the driver's live measured
q — it is the real confirmation of motion) plus `fraction_complete`.

**Goal validation** happens in `handle_goal` before the Supervisor sees anything.
A goal is REJECTED if the trajectory has no points, or if any point's `positions`
length is not exactly 7. (The mapping layer separately zero-fills / truncates as a
memory-safety net, but a malformed goal never gets that far.) Only the `.position`
field of each `JointTolerance` is used; a value `< 0` disables that joint's guard.

`go_to_ee_pose`, `go_to_joint_config` and `go_to_preset` describe a *goal* rather
than a trajectory: each plans a collision-free path via the external cuRobo node
and executes it through the same Supervisor. They share one lifecycle, so their
Result/Feedback and their cancel behaviour are identical — see
[`docs/guide-goto-actions.md`](docs/guide-goto-actions.md) for all three
(`go_to_ee_pose` alone is also covered in
[`docs/guide-goto-ee-pose.md`](docs/guide-goto-ee-pose.md)).

Both joint-space actions plan through cuRobo's `plan_to_joints`; `go_to_preset`
just resolves a name to 7 joint angles first, from the `preset_names` /
`presets.<name>` parameters.

### Published topics

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | `SensorDataQoS` (**best-effort**) | `joint_1`..`joint_7`; `position`/`velocity`/`effort` all filled. Free-running from the pump thread, ~100 Hz. |
| `control_status` | `kinova_gen3_interfaces/ControlStatus` | reliable, **transient_local** (latched) | Who may command the arm: owner, `generation`, `estopped`, `rejected_count`. Published **on change**, so a late or reconnecting client learns the current state immediately. |
| `ee_state` | `kinova_gen3_interfaces/EeState` | `SensorDataQoS` (**best-effort**) | The Cartesian sibling of `joint_states`: tool pose and twist, same pump tick, same rate. |
| `stream_status` | `kinova_gen3_interfaces/StreamStatus` | reliable, **transient_local** (latched) | What the streaming tier is doing: `open`, `controller`, `channels`, `timeout_s`, `rejected_count`. `open`, `timeout_s` and `rejected_count` come from core via `StreamSink::on_query_stream()`, so a session torn down on deadline expiry shows up immediately rather than as this node's guess. Published **on change**. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | default | REP 107, 1 Hz, via `diagnostic_updater`. Two tasks: `kinova_gen3_node: Arbitration` (ERROR while e-stopped, WARN when unowned in enforced mode) and `kinova_gen3_node: Arm` (ERROR on an arm fault, STALE before any feedback arrives). |

Because `joint_states` and `ee_state` are best-effort, CLI subscribers must match it:
`ros2 topic echo --qos-reliability best_effort /joint_states`.

**`ee_state` pose and twist are model-derived, not read from the arm.** Pose is `fk(q)`
and twist is `J(q)·qd`, computed together in the driver's pump — so they agree with each
other and with the model cuRobo plans against. The arm's own `tool_pose`/`tool_twist`
feedback exists but sits in KORTEX's configured tool frame, which carries an offset the
URDF does not know about; mixing the two would give a pose and a twist that disagree by
an unknown transform. The frame is **`LOCAL_WORLD_ALIGNED`** — world-aligned at the tool,
not the body frame.

There is deliberately **no wrench**. The driver has no force estimate of its own, and the
arm's `tool_external_wrench` is in that same tool frame; publishing a zero in sim or a
frame-mismatched value on hardware would be worse than publishing nothing.

The same judgement applies to the gripper, once it is published: its effort is a
normalized 0..1 fraction derived from motor current, and `sensor_msgs/JointState.effort`
is documented as N·m or N. So the gripper's **effort stays NaN in `/joint_states`
permanently** — the fraction belongs on a future `/gripper_state` where its units can be
stated, next to the raw current in amps. There is no force *feedback* on this hardware to
report: the gripper's `force` is a current ceiling on the command side, and
`MotorFeedback` has no force field at all.

### Subscribed topics

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/estop` | `kinova_gen3_interfaces/EStop` | reliable, **volatile** | Broadcast emergency stop. `engaged: true` stops the arm, `false` clears. Global (leading `/`), and **any node may publish either**. |
| `/setpoint/joint_position` | `kinova_gen3_interfaces/JointSetpoint` | best-effort, depth 1 | Joint angles, **rad**. |
| `/setpoint/joint_velocity` | `kinova_gen3_interfaces/JointSetpoint` | best-effort, depth 1 | Joint rates, **rad/s**. |
| `/setpoint/joint_torque` | `kinova_gen3_interfaces/JointSetpoint` | best-effort, depth 1 | Joint torques, **N·m**. |
| `/setpoint/pose` | `kinova_gen3_interfaces/PoseSetpoint` | best-effort, depth 1 | Target tool pose, base frame. |
| `/setpoint/twist` | `kinova_gen3_interfaces/TwistSetpoint` | best-effort, depth 1 | Target tool twist, base frame, `[linear; angular]`. |
| `/setpoint/wrench` | `kinova_gen3_interfaces/WrenchSetpoint` | best-effort, depth 1 | Target tool wrench. **No controller consumes this** — setpoints are dropped with a throttled warning. |

The three `JointSetpoint` topics share one message shape and the **topic** carries the units.
All six are subscribed unconditionally, but a setpoint is only applied while a matching
streaming session is open and its token matches — see [Streaming](#streaming) for the
controller-to-channel map and the open/close handshake.

`/estop` is deliberately *not* latched. A `transient_local` subscription is
incompatible with a volatile publisher, which is what `ros2 topic pub` and rqt
produce — requesting durability would make an operator's e-stop silently fail to
connect. This works:

```bash
ros2 topic pub --once /estop kinova_gen3_interfaces/msg/EStop \
  "{engaged: true, source: 'cli', reason: 'testing'}"
```

**Staleness is asymmetric, and both branches fail toward "the arm stays stopped":**
`engaged: true` is never age-checked (a stale stop is still honoured), while
`engaged: false` is refused if older than `estop_clear_max_age_s` — which stops a
`ros2 bag` replay from re-enabling a stopped arm. An unstamped clear (all-zero
stamp, what `ros2 topic pub` sends) is accepted with a warning.

### Control-ownership services

| Service | Type | Notes |
|---|---|---|
| `acquire_control` | `AcquireControl` | Mints a token. **SEIZES** — succeeds even when another client holds the arm, halting their in-flight motion (settled `-9`). |
| `release_control` | `ReleaseControl` | Refused unless the token matches the current owner. |
| `revoke_control` | `RevokeControl` | Operator override, no token. The recovery path for a crashed owner, since ownership has no lease. |

```bash
# acquire, then put the returned token on every goal
ros2 service call /acquire_control kinova_gen3_interfaces/srv/AcquireControl \
  "{owner_id: 'orchestrator'}"
```

Every motion-commanding message carries a `uint8[16] token`; nothing that *stops*
the arm or reads state does. With `arbitration_mode:=disabled` (the default) the
token is ignored, so existing clients need no changes.

> **Arbitration is cooperative coordination, not authorization.** `grant()` verifies
> nothing about the caller, so anyone can acquire a valid token; it prevents mistakes
> between known, cooperating participants, not deliberate actors. By operational
> contract `acquire_control` is called by the task orchestrator and nothing else.
> ROS action cancels are unauthenticated by protocol and cannot be otherwise. If the
> domain ever contains unknown actors, the answer is SROS2 / DDS Security, not more
> tokens. See `docs/superpowers/specs/2026-08-29-ros2-arbitration-tier-design.md`.

`set_gains` and `query_state` exist on the core's `CommandSink` but are still not
exposed as ROS2 services.

### Streaming

Teleop and reactive control drive the arm through a **session**: you name a
*controller* (a control law), and the driver replies with the *channels* (topics)
to publish on.

| Service | Type | Notes |
|---|---|---|
| `list_controllers` | `ListControllers` | Call this **first** — see the discovery note below. |
| `open_stream` | `OpenStream` | `controller, timeout_s, token` → `accepted, channels[], error_code, message` |
| `close_stream` | `CloseStream` | `token` → `closed, message` |

| Controller | Channel | Available |
|---|---|---|
| `joint_position` | `/setpoint/joint_position` | yes |
| `joint_impedance` | `/setpoint/joint_position` | yes |
| `ee_pose_impedance` | `/setpoint/pose` | yes — compliant; in-loop IK via `JointImpedanceMode` |
| `ee_pose_position` | `/setpoint/pose` | yes — **stiff**; no compliance, full servo authority |
| `joint_torque` | `/setpoint/joint_torque` | yes |
| `joint_velocity` | `/setpoint/joint_velocity` | yes — **stiff by contract**: tracks the rate, does not yield to contact |
| `ee_twist` | `/setpoint/twist` | yes — damped least squares with null-space posture |
| `cartesian_impedance` | `/setpoint/pose`, `/setpoint/wrench` | no — needs `CartesianImpedanceMode` in the `Supervisor` and a `kEeWrench` kind |

`available` is computed live from core's `pair_supported()`, so these rows light up
when core grows the mode. That is not theoretical: `joint_velocity` and `ee_twist`
flipped to available when core landed `JointVelocityMode`, with no change on this side
beyond the test expectation.

**Velocity and `ee_pose_position` are stiff.** Neither yields to contact — the servo
chases the command at full authority, and nothing absorbs a mistake. `joint_impedance`
and `ee_pose_impedance` are the compliant options.

Note that `SimTransport` is a static echo with no velocity plant, so a velocity or twist
session commands correctly in sim but produces **no motion**. That the setpoints are
landing is still observable: they refresh the session deadline, so `/stream_status` stays
`open` past `timeout_s` only while they are actually arriving.

```bash
ros2 service call /acquire_control kinova_gen3_interfaces/srv/AcquireControl "{owner_id: 'teleop'}"
ros2 service call /list_controllers kinova_gen3_interfaces/srv/ListControllers "{}"
# create your publisher and let discovery settle BEFORE opening -- see below
ros2 service call /open_stream kinova_gen3_interfaces/srv/OpenStream \
  "{controller: 'joint_impedance', timeout_s: 0.1, token: [...]}"
# publish on the returned channel, faster than timeout_s
ros2 service call /close_stream kinova_gen3_interfaces/srv/CloseStream "{token: [...]}"
```

**Create your publisher before you open.** DDS discovery can take hundreds of
milliseconds and a session deadline is typically 100 ms. Open first and your early
setpoints go nowhere, so the session expires before it ever drives the arm. This is
why `list_controllers` reports channels at all.

**Four rules that bite:**

- One session at a time; a second `open_stream` is refused, not queued.
- The controller is fixed for the session's lifetime. Changing what you stream means
  close-then-reopen, which re-pays the 250 ms mode settle.
- Streams and trajectory goals refuse each other in both directions.
- A setpoint on the wrong channel is dropped, counted, and **does not refresh the
  deadline** — publishing hard on the wrong topic will not keep a session alive.

**Setpoint topics are best-effort, depth 1.** That is core's semantics, not a
tuning choice: setpoints are absolute and latest-wins, so a dropped intermediate is
correct and a *late* one is harmful. CLI subscribers must match the QoS:

```bash
ros2 topic echo --qos-reliability best_effort /setpoint/joint_position
```

`/stream_status` (reliable, latched, on change) reports core's actual session — not
this node's record of it — so a session torn down on deadline expiry or by an e-stop
shows up immediately. Its `rejected_count` counts setpoints the **session** refused
(wrong channel); token failures are counted by the Arbiter and appear on
`/control_status` instead.

`/setpoint/wrench` exists so the surface is complete, but no controller consumes it
yet; messages are dropped with a throttled warning.

### Launch files

There are none. The node takes plain CLI args and is started with `ros2 run`;
adding a launch file has not been needed yet.

## Robot model and TF

`kinova_gen3_description` holds the robot model and the launch plumbing. Start everything
with:

```bash
ros2 launch kinova_gen3_description bringup.launch.py sim:=true
# or against the arm:
ros2 launch kinova_gen3_description bringup.launch.py sim:=false ip:=192.168.1.10
```

That runs the driver and `robot_state_publisher` on the **same model**, which is the
point of the package. For TF only, without owning the arm:

```bash
ros2 launch kinova_gen3_description description.launch.py
```

### The model is generated, not vendored

`urdf/kinova_gen3.urdf.xacro` composes `kortex_description`'s arm with
`robotiq_description`'s 2F-85, and CMake expands it at build time into **two** URDFs:

| file | DOF | for |
|---|---|---|
| `kinova_gen3_7dof.urdf` | 7 | the driver (core's `Dynamics` asserts `nv == 7`) |
| `kinova_gen3.urdf` | 13 | `robot_state_publisher` by default (`articulated:=true`) |

Two, because they cannot be one:

- **Core's `Dynamics` asserts `nv == 7`** and aborts otherwise
  (`URDF nv=13 != kNumJoints=7`). `JointVec` is a fixed-size 7-vector.
- **`robot_state_publisher` DOES derive `<mimic>` joint transforms from the joint they
  mimic** — verified 2026-09-03 with a two-joint URDF: publishing only the driver joint
  moved the mimic link by exactly -0.6 rad for a +0.6 rad drive. The articulated model
  has 13 movable joints but only **eight independent** ones — seven arm joints plus
  `robotiq_85_left_knuckle_joint` — and the driver publishes all eight as of the gripper
  tier, so `robot_state_publisher` derives the other five and `articulated` now defaults
  to `true`.

Freezing the gripper drops its degrees of freedom, **not its mass** — Pinocchio lumps a
fixed joint's body into its parent — so gravity compensation is unaffected. That is
almost certainly why the hand-edited model this replaces had every Robotiq joint fixed.

It is safe on a gripper-less build: if the model was built with `gripper:=false`, the
knuckle joint is not in it and `robot_state_publisher` ignores joint states for joints
it does not know.

### The invariant

**The driver publishes `joint_1 … joint_7` and the model must agree.** It did not
before: the old URDF used `gen3_joint_1 … gen3_joint_7`, so `robot_state_publisher`
matched nothing and held TF at the default configuration while the arm really moved.
Nothing errored. `test_tf_updates.py` asserts the transform *changes*, not merely that
it exists.

`--ee-frame` exists for the same reason. `Dynamics` resolves the EE frame by name and
throws if it is absent; core defaults to `gen3_end_effector_link` for its own tests,
while the generated model uses `end_effector_link`, which the launch passes.

### Known gaps

- The **wrist camera and its 0.5 kg mount are not in the generated model** — they were in
  the hand-edited one and upstream cannot know about them. `test_model_parity.py` pins
  the resulting 0.436 kg difference so it cannot be forgotten; if the physical arm
  carries them, gravity compensation is under-modelled by that much at the wrist.
- The **gripper joint is not published**, so the gripper renders at its default opening.
- Model configuration (`gripper`, `camera`, `prefix`) is a **build-time** xacro arg, not
  a launch argument.

## Node arguments

| Flag | Default | Meaning |
|---|---|---|
| `--sim` | off | Use `SimTransport` instead of the real arm. |
| `--ip <addr>` | — | Arm IP; required in real mode. Ignored with `--sim`. |
| `--urdf <path>` | `models/gen3_7dof_2f85.urdf` | Relative to the **cwd**, so run from the core checkout (which ships `models/`). |
| `--cpu <n>` | `-1` (no pin) | CPU to pin the RT thread to. |
| `--rt-priority <n>` | `80` | SCHED_FIFO priority for the RT thread. |
| `--rate <hz>` | `1000.0` | RT loop rate. |
| `--max-ref-speed <rad/s>` | URDF velocity limits | Cap on how fast the position-mode *reference* may move, applied per joint. See below. |

`--max-ref-speed` is worth understanding before you change it. `JointPositionParams`
defaults to 0.5 rad/s on every joint — a conservative bring-up value that
`trajectory_run` overrides from a flag — while the Gen3's URDF limits are 1.3963
(j1–4) and 1.2218 (j5–7). Left at the default, the node throttles every joint to
roughly 0.4x of what the arm can do, so any trajectory planned near the real
limits (anything cuRobo produces) is tracked late, and by a *different* amount per
joint — the joints stop arriving together. It also aborts goals: the divergence
guard compares measured q against the **planned** sample while the mode commands
the rate-limited reference, so the throttle manufactures the very divergence that
trips `PATH_TOLERANCE_VIOLATED`. The node therefore seeds this from the URDF and
logs the result at startup; pass the flag only to go deliberately *slower*, e.g.
for a cautious first on-robot run.

### ROS parameters

| Parameter | Default | Meaning |
|---|---|---|
| `arbitration_mode` | `disabled` | `enforced` \| `disabled`. **Read-only**, set at launch. |
| `estop_clear_max_age_s` | `1.0` | Age beyond which an `/estop` *clear* is ignored. `<= 0` disables the check. Engaging is never age-checked. |

```bash
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf \
  --ros-args -p arbitration_mode:=enforced
```

`arbitration_mode` is **read-only on purpose**: core's `ArbitrationMode` is an
`Arbiter` constructor argument with no setter, so a dynamic parameter would appear
to work and silently do nothing. It defaults to `disabled` so every existing client
and script keeps working unchanged — which costs no safety, because `estop()`
latches over *both* modes; `kDisabled` is the one thing e-stop does not bypass.

Build option `KINOVA_ENABLE_KORTEX` (default `OFF`) selects whether the real
`KortexTransport` path is compiled in. With it OFF the node is sim-only and exits
with an error if launched without `--sim`.

## Build

Both packages are colcon/ament; the core is vendored into the same workspace
`src/` and found via `find_package(kinova_lowlevel CONFIG REQUIRED)` (the core
exports `kinova_lowlevelConfig.cmake`). `kinova_gen3.repos` documents the intended
source pin (core `main`) and is what the **container** build vcs-imports; the
bare-metal dev loop rsyncs a local core working tree instead, because abra has no
GitHub key (and that way it picks up uncommitted core changes).

Everything builds **and runs on abra** (aarch64) — same host, so the absolute
paths baked into the core's exported target stay valid.

```sh
# from muk — rsync core + this repo to abra, then colcon build there (sim by default)
bash scripts/abra_colcon.sh
bash scripts/abra_colcon.sh --packages-select kinova_gen3_ros2      # extra args pass through
```

Real-arm (KORTEX-linked) build — **always pass the flag explicitly**, since the
CMake cache persists `KINOVA_ENABLE_KORTEX` across rebuilds:

```sh
bash scripts/abra_colcon.sh --cmake-args \
  -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64
# and back to sim-only when done on the arm:
bash scripts/abra_colcon.sh --cmake-args -DKINOVA_ENABLE_KORTEX=OFF
```

Sanity check for which mode got built: the KORTEX binary is ~9.7 MB (vs ~1.5 MB
sim) and `strings` shows `KortexTransport::connect`.

Note `set_target_properties(... INSTALL_RPATH_USE_LINK_PATH TRUE)` in
`kinova_gen3_ros2/CMakeLists.txt` — the core is a static lib that links pinocchio
shared objects under the cmeel prefix with no RPATH of its own; without this the
installed node builds fine but can't find `libpinocchio_*.so` at runtime.

## Container

`docker/Dockerfile` builds the whole workspace — core driver included — into one
image for the Jetson AGX Orin. It follows the RAMMP-Software module conventions
(`ros:humble` + Cyclone DDS + an entrypoint that sources ROS and then the
workspace) but stays standalone: it does **not** require a locally-built
`rammp-base:humble`. Build it **on the Orin** (arm64, and the KORTEX SDK lives
there); the `Makefile` wraps the flags.

```sh
make build                 # sim image  -> kinova-gen3-ros2:humble
make sim                   # run it, foreground
make e2e                   # two-goal integration check + SCHED_FIFO assertion
make shell                 # poke around inside
```

Real arm — **attended only**, per `docs/on-robot-runbook.md`:

```sh
make stage-kortex          # copy the aarch64 KORTEX SDK into docker/vendor/
make real IP=192.168.1.10  # KORTEX-enabled build, then run against the arm
```

Three things about this image are load-bearing:

- **pinocchio is pinned to `pip install pin==3.9.0`**, matching the version
  validated on the Jetson, and lands at the same cmeel prefix the bare-metal
  build uses. It is deliberately *not* `ros-humble-pinocchio`, which is 4.0.0 on
  Humble arm64 — a major version ahead of what the core is validated against.
  That is also why `rosdep install` runs with `--skip-keys pinocchio`: the core's
  `package.xml` declares the dep, and rosdep would otherwise install the apt
  version over the pinned wheel.
- **The KORTEX SDK must be in the build context**, not bind-mounted:
  `libKortexApiCpp.a` is linked statically at build time. `make stage-kortex`
  rsyncs it from `~/kortex_api_2.8.0_aarch64` into `docker/vendor/`, which is
  gitignored — the SDK is proprietary and must never be committed.
- **The container runs the node binary directly, not via `ros2 run`.** `ros2 run`
  forks the real binary as a child of a Python wrapper, so the SIGTERM from
  `docker stop` would hit the wrapper and the node would be SIGKILLed with no
  `safe_shutdown()`. Exec'd directly it is PID 1 and its SIGTERM handler runs.

The image sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, so **anything talking to
it must too** — a host shell left on the Humble default (Fast DDS) will see no
nodes and give you a silent, confusing nothing:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # on the host, before ros2 CLI
```

Run flags that matter (all encoded in the Makefile): `--network host --ipc host`
so DDS discovery and its shared-memory transport work across the container
boundary, and `--cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1` so
`mlockall` and `SCHED_FIFO`(80) succeed inside the container. Full `--privileged`
is not needed. Verify the RT part with `chrt -p 1` inside the container — it
should report `SCHED_FIFO` priority 80.

**Core pinning is a node argument, not a Docker flag.** abra boots
`isolcpus=11 nohz_full=11 rcu_nocbs=11` and the core driver's `scripts/rt_setup.sh`
defaults `RT_CORE=11`. `isolcpus` removes that core from the scheduler's load
balancing, so a thread reaches it *only* via explicit affinity — and `enable_rt()`
guards its `sched_setaffinity` on `cpu >= 0`, which `--cpu` is the only way to set.
Omit `--cpu` and the 1 kHz loop runs on the general cores forever, even though
`chrt` still cheerfully reports `SCHED_FIFO`/80. The Makefile passes
`--cpu $(RT_CORE)` (default 11) on every run target. Verify with:

```sh
docker exec <container> taskset -pc 1     # expect "current affinity list: 11"
```

Do **not** use Docker's `--cpuset-cpus` for this. That confines the entire
container — rclcpp executor and telemetry drain included — to the isolated core,
which is the opposite of what the isolation buys you. `--cpu` pins only the RT
loop, because `enable_rt()` runs inside `RtExecutor::run()` on the main thread,
after `bringup_node` has already spawned the non-RT threads.

## Run

Sim, on abra:

```sh
source /opt/ros/humble/setup.bash
source /tmp/kinova-ros2-ws/install/setup.bash
cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver          # for models/
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf
# expect: "kinova_gen3_node up (sim); action: /execute_joint_trajectory"
```

Real arm (attended only — follow `docs/on-robot-runbook.md`):

```sh
ros2 run kinova_gen3_ros2 kinova_gen3_node --ip 192.168.1.10 --urdf models/gen3_7dof_2f85.urdf
```

Send a goal with the test client. It **seeds waypoint 0 from the live measured
`/joint_states` pose**, then moves only the requested joints by the requested
deltas — so on a real arm at any pose it commands a small *local* move, never a
jump to zero:

```sh
python3 <ws>/src/kinova_gen3_ros2/kinova_gen3_ros2/test/send_trajectory.py \
  --joint 6 --mode position --delta 0.10 --dur 1.2 --path-tol 0.2 --expect 0

# coordinated multi-joint: comma-lists, one delta per joint
# (put a positive delta first — argparse reads a leading '-' value as a flag)
python3 .../send_trajectory.py --joint 5,6 --delta 0.4,-0.4 --dur 2.5 --expect 0
```

Client flags: `--joint` (comma-list, 0..6), `--delta` (comma-list, matching
length), `--mode position|impedance`, `--dur` seconds, `--path-tol` (`<0`
disables), `--expect <error_code>` (required; the process exits non-zero if the
result code doesn't match, which is what makes it usable as a test).

Killing the node: `ros2 run` forks the real binary as a child of a Python
wrapper, so killing the wrapper's PID orphans the RT node. Reap the wrapper, then
`pkill -TERM -f .../kinova_gen3_node` — see `scripts/abra_e2e_sim.sh`.

## Test

Unit (`colcon test`): `message_mapping_test` covers goal→`TrajectoryGoal`
mapping for position and impedance modes, tolerance mapping (empty ⇒ disabled),
the short/long `positions` safety net (zero-fill / take-first-seven), and result
mapping.

Sim end-to-end — run **on abra**, launches the node, sends two goals, asserts
both, and cleans up:

```sh
bash scripts/abra_e2e_sim.sh
# success case: small position-mode move          -> error_code 0
# divergence case: big move, tight path tolerance -> error_code -4 (PATH_TOLERANCE_VIOLATED)
```

This exercises the whole pipe: client → action server → `CommandSink` →
Supervisor → sampler → `SimTransport` → driven ports → result.

On-robot testing is **attended only** and follows `docs/on-robot-runbook.md`
(small / slow / distal, e-stop in hand). Append every run to that file's Runs
section.
