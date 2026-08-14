# kinova_arm_ros2

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
| `kinova_arm_interfaces` | `ament_cmake` + `rosidl` | `ExecuteJointTrajectory.action`, `JointImpedanceGains.msg`. Interface definitions only. |
| `kinova_arm_ros2` | `ament_cmake` | `message_mapping` + `ros2_backend` libraries and the `kinova_arm_node` executable. |

```
kinova_arm_interfaces/
  action/ExecuteJointTrajectory.action
  msg/JointImpedanceGains.msg
kinova_arm_ros2/
  include/kinova_arm_ros2/{ros2_backend,message_mapping}.h
  src/message_mapping.cpp     ROS2 msg <-> kinova::interface value types (no rclcpp)
  src/ros2_backend.cpp        the ONLY unit that includes rclcpp/rclcpp_action
  src/bringup_node.cpp        DI wiring + main()
  test/message_mapping_test.cpp   gtest (runs under `colcon test`)
  test/send_trajectory.py         rclpy action client used as the integration harness
kinova_arm.repos            vcs source list vendoring the core into the colcon ws
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

Threading in `kinova_arm_node`: the RT executor runs the 1 kHz loop on the **main
thread**; the rclcpp `SingleThreadedExecutor` and the telemetry-ring drain each
get their own thread. Everything ROS2 is non-RT and reaches the loop only through
the Supervisor's existing lock-free seams. `SIGINT` and `SIGTERM` both set the
stop flag, which unblocks the RT loop and triggers `sup.stop()` +
`transport.safe_shutdown()`.

## ROS2 interface

Node name: **`kinova_arm_node`**.

### Action servers

| Name | Type |
|---|---|
| `execute_joint_trajectory` | `kinova_arm_interfaces/action/ExecuteJointTrajectory` |

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

### Published topics

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | `SensorDataQoS` (**best-effort**) | `joint_1`..`joint_7`; `position`/`velocity`/`effort` all filled. Free-running from the pump thread, ~100 Hz. |

Because the QoS is best-effort, CLI subscribers must match it:
`ros2 topic echo --qos-reliability best_effort /joint_states`.

### Subscribed topics / services

None. `set_gains` and `query_state` exist on the core's `CommandSink` but are not
yet exposed as ROS2 services — that's the next plan.

### Launch files

There are none. The node takes plain CLI args and is started with `ros2 run`;
adding a launch file has not been needed yet.

## Node arguments

| Flag | Default | Meaning |
|---|---|---|
| `--sim` | off | Use `SimTransport` instead of the real arm. |
| `--ip <addr>` | — | Arm IP; required in real mode. Ignored with `--sim`. |
| `--urdf <path>` | `models/gen3_7dof_2f85.urdf` | Relative to the **cwd**, so run from the core checkout (which ships `models/`). |
| `--cpu <n>` | `-1` (no pin) | CPU to pin the RT thread to. |
| `--rt-priority <n>` | `80` | SCHED_FIFO priority for the RT thread. |
| `--rate <hz>` | `1000.0` | RT loop rate. |

Build option `KINOVA_ENABLE_KORTEX` (default `OFF`) selects whether the real
`KortexTransport` path is compiled in. With it OFF the node is sim-only and exits
with an error if launched without `--sim`.

## Build

Both packages are colcon/ament; the core is vendored into the same workspace
`src/` and found via `find_package(kinova_lowlevel CONFIG REQUIRED)` (the core
exports `kinova_lowlevelConfig.cmake`). `kinova_arm.repos` documents the intended
source pin (core `main`) and is what the **container** build vcs-imports; the
bare-metal dev loop rsyncs a local core working tree instead, because abra has no
GitHub key (and that way it picks up uncommitted core changes).

Everything builds **and runs on abra** (aarch64) — same host, so the absolute
paths baked into the core's exported target stay valid.

```sh
# from muk — rsync core + this repo to abra, then colcon build there (sim by default)
bash scripts/abra_colcon.sh
bash scripts/abra_colcon.sh --packages-select kinova_arm_ros2      # extra args pass through
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
`kinova_arm_ros2/CMakeLists.txt` — the core is a static lib that links pinocchio
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
make build                 # sim image  -> kinova-arm-ros2:humble
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

## Run

Sim, on abra:

```sh
source /opt/ros/humble/setup.bash
source /tmp/kinova-ros2-ws/install/setup.bash
cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver          # for models/
ros2 run kinova_arm_ros2 kinova_arm_node --sim --urdf models/gen3_7dof_2f85.urdf
# expect: "kinova_arm_node up (sim); action: /execute_joint_trajectory"
```

Real arm (attended only — follow `docs/on-robot-runbook.md`):

```sh
ros2 run kinova_arm_ros2 kinova_arm_node --ip 192.168.1.10 --urdf models/gen3_7dof_2f85.urdf
```

Send a goal with the test client. It **seeds waypoint 0 from the live measured
`/joint_states` pose**, then moves only the requested joints by the requested
deltas — so on a real arm at any pose it commands a small *local* move, never a
jump to zero:

```sh
python3 <ws>/src/kinova_arm_ros2/kinova_arm_ros2/test/send_trajectory.py \
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
`pkill -TERM -f .../kinova_arm_node` — see `scripts/abra_e2e_sim.sh`.

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
