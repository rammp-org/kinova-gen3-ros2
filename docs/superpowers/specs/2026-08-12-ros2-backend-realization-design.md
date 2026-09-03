# ROS2 Backend — Realization Design (Plan 3, minimal end-to-end)

**Date:** 2026-08-12
**Status:** Approved (design locked; realizes the already-approved interface + build specs)
**Repo:** `rammp-org/kinova_gen3_ros2` (this repo) — the ROS2 Humble frontend.

## What this is

The concrete realization of the ROS2 frontend for the Kinova Gen3 low-level
driver. The *architecture* is already fixed by two approved documents in the core
repo (`kinova-gen3-driver`):

- **Interface design spec** (`docs/superpowers/specs/2026-08-10-arm-driver-interface-design.md`)
  — Component 2 (`ExecuteJointTrajectory` action, message shapes), Component 5
  (`Ros2Backend`), Layer A ports, telemetry, RT-safety invariants.
- **Build-integration strategy** (`docs/superpowers/specs/2026-08-12-ros2-build-integration-strategy.md`)
  — team decisions: core stays plain-CMake/ROS-free; **b1** (core exports
  `kinova_lowlevelConfig.cmake`, ROS2 side `find_package`s it); ROS2 layer in a
  **separate repo** vendoring the core via `.repos`.

This document adds only the *realization* specifics: how the pieces land in this
repo, the combined KORTEX+ROS2 build, and the milestones. It does not re-decide
architecture. **Scope of Plan 3: the `ExecuteJointTrajectory` action path only,
taken end-to-end through to an attended real-arm run. The `JointState` stream and
the set-gains / query-state services are a fast-follow plan.**

## Dependency status (as of 2026-08-12)

- The core's **Layer A ports + Supervisor + b1 export** are implemented on the
  core repo branch `feat/interface-supervisor-ports` (Plan 2, merge-ready, **not
  yet merged to `main`**). Plan 3 vendors the core at **that branch** until Plan 2
  merges, then flips the `.repos` ref to `main`.
- The core builds two ways today: sim (default) and KORTEX
  (`-DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=<aarch64 SDK>`). Both are driven
  through colcon in this plan via `--cmake-args`.

## Repo structure

```
kinova_gen3_ros2/                       (this repo)
  kinova_gen3.repos            vcs source list: vendors kinova-gen3-driver (the core)
                              into the colcon workspace src/. Pinned to the core
                              branch until Plan 2 merges (then main).
  kinova_gen3_interfaces/      ament_cmake — interface definitions ONLY
    package.xml               <build_type>ament_cmake</build_type>; member_of_group
                              rosidl_interface_packages; depends trajectory_msgs,
                              control_msgs, builtin_interfaces, action_msgs, std_msgs
    CMakeLists.txt            rosidl_generate_interfaces(...)
    action/ExecuteJointTrajectory.action
    msg/JointImpedanceGains.msg
  kinova_gen3_ros2/            ament_cmake — the backend + bring-up node
    package.xml               depends rclcpp, rclcpp_action, kinova_gen3_interfaces,
                              kinova_lowlevel, sensor_msgs (stream is later; dep is fine)
    CMakeLists.txt            find_package(kinova_lowlevel CONFIG REQUIRED) + rclcpp*
    include/kinova_gen3_ros2/ros2_backend.h
    src/ros2_backend.cpp      the ONLY unit that includes rclcpp/rclcpp_action
    src/bringup_node.cpp      DI wiring + spin (main)
  test/                       Python test client(s) driving the action
    send_trajectory.py        rclpy action client: sends ExecuteJointTrajectory,
                              prints feedback, asserts the result code
```

## The `ExecuteJointTrajectory` action (from the interface spec, Component 2)

Custom action composed from the standard FJT messages; self-contained mode
selection. Goal carries the standard `trajectory_msgs/JointTrajectory` +
`control_msgs/JointTolerance[]` path/goal tolerances + our additions
(`control_mode`, `preemption`, `gains`, `sender_id`). Result mirrors FJT
(`error_code` + string + `final_error`). Feedback mirrors FJT
(desired/actual/error + `fraction_complete`). The exact `.action`/`.msg` field
lists are in the interface spec, Component 2 — this plan transcribes them.

## Component: `Ros2Backend` (realizes Component 5)

The single rclcpp-including unit. It:

- **implements the driven ports** `ActionServerPort` (`publish_feedback`,
  `settle`) and, later, `StreamPort` — the Supervisor calls these to push
  feedback/results out; the backend maps the plain value types → ROS2 messages
  (field copies, since the value types mirror FJT content) and drives the rclcpp
  action goal handle (`publish_feedback` / `succeed` / `abort` / `canceled`).
- **owns an rclcpp action server** for `ExecuteJointTrajectory`; its
  `handle_goal` / `handle_accepted` / `handle_cancel` callbacks translate the
  inbound ROS2 goal → the plain `TrajectoryGoal` value type and call the
  Supervisor's `CommandSink` (`on_trajectory_goal` / `on_trajectory_accepted` /
  `on_trajectory_cancel`). Goal UUID → the `GoalId` (16 bytes).
- holds a map `GoalId → rclcpp goal handle` so `settle`/`publish_feedback` (driven
  by the Supervisor on its sampler thread) find the right handle. Access to that
  map is mutex-guarded (backend callbacks run on the rclcpp executor thread;
  Supervisor drives the driven ports from its sampler thread).

**GoalId ↔ UUID mapping caveat.** The supervisor's cancel is id-agnostic in v1
(core design note): it cancels whatever is active/queued. The backend maps the
rclcpp per-goal cancel request onto the supervisor's `on_trajectory_cancel`; the
result (`PREEMPTED`) is routed to whichever goal handle the supervisor settles.
This is acceptable for the single-commander v1 and is flagged for the arbitration
follow-on.

## Component: bring-up node (DI wiring, `main`)

Mirrors the wiring proven by `trajectory_run` / `teleop_socket_server`, but the
command source is the action server instead of a CLI/socket:

1. Parse args (`--sim` | `--ip <addr>`, `--urdf`, RT params).
1. Construct the transport (`SimTransport` or `KortexTransport`), wrap in
   `FeedbackTap` + `Seqlock<JointFeedback>`.
1. Construct `Dynamics` (one for the RT/mode side, one for the pump),
   `JointPositionMode` + `JointImpedanceMode`, `SampleRing`, `RtExecutor`.
1. Construct `Ros2Backend`; construct the `Supervisor` against the backend's
   driven ports; register the supervisor's `CommandSink` with the backend.
1. `supervisor.start()`; run the RT loop on the main thread (`exec.run`), spin the
   rclcpp executor + telemetry drain on their own threads. Clean shutdown joins
   all non-RT threads before `transport.safe_shutdown()`.

**RT-safety carries over unchanged:** the rclcpp executor, action callbacks,
sampler, and pump are all non-RT; they reach the 1 kHz loop only through the
Supervisor's existing lock-free seams. Nothing new runs on the RT thread.

## The combined KORTEX+ROS2 build (the one genuinely new build wrinkle)

Everything builds **and runs on abra** (aarch64), so absolute paths baked into
the core's exported target are valid — no cross-machine relocation problem.

- **Sim build:** `colcon build` — core built without the KORTEX flag; the node
  links `SimTransport`. This is the CI/default path and Milestone A.
- **Real-arm build:** `colcon build --cmake-args -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64` — colcon passes these to
  the core's cmake package, so `KortexTransport` is compiled and the static core
  carries the KORTEX link requirement transitively into the node. Because the
  build+run host is abra, the KORTEX absolute path in the exported target resolves
  correctly. RT privileges (SCHED_FIFO etc.) come from the core's `rt_system` at
  node startup, exactly as `trajectory_run` does.

## Milestones (each independently testable)

- **A — sim end-to-end (the proof, no hardware):** build the workspace (sim);
  the Python client sends a real multi-waypoint `ExecuteJointTrajectory` (position
  mode) → assert feedback streams and the result settles `SUCCESSFUL`; a
  forced-divergence goal (path tolerance tight, static sim) settles
  `PATH_TOLERANCE_VIOLATED`. This validates the whole pipe:
  client → action server → CommandSink → Supervisor → sampler → SimTransport →
  driven ports → result.
- **B — combined KORTEX+ROS2 build** green on abra (`--cmake-args` KORTEX path).
- **C — attended real-arm run:** bring-up node against `KortexTransport`; the user
  sends a small / slow / single-joint trajectory from the Python client, e-stop in
  hand (same posture and safeguards as the Plan 1 on-robot step — mode leash, slow
  cap, live divergence guard). Gated behind A + B passing.

## Testing strategy

- **Interfaces build** (Milestone A prerequisite): the `.action`/`.msg` generate
  and the messages are importable from `rclpy`/`rclcpp`.
- **Sim integration** (A): the Python action client is the test harness; it
  asserts feedback arrival, `fraction_complete` progression, and terminal result
  codes for both the success and forced-divergence cases. Runs headless on abra.
- **RT-safety:** the core's `RtSafety` supervisor-in-the-loop gate already covers
  the sampler+pump against `SimTransport`; the node adds only non-RT rclcpp work,
  so no new RT gate is required for the sim path. The real-arm run (C) reports
  `faults`/`dropped`/`majflt` from the telemetry drain, as `trajectory_run` does.
- **On-robot (C):** attended only, per the core's integration runbook; never run
  unattended.

## Out of scope (fast-follow, next plan)

- `JointState` live-state stream (`StreamPort`) and the diagnostic topic.
- `set_gains` / `query_state` services.
- cuRobo as the real client (this plan uses a hand-written Python test client).
- Multi-sender arbitration (hooks only, per the interface spec).
- Flipping the `.repos` ref to `main` (done once Plan 2 merges).
