# Handoff — cuRobo-backed high-level action tier (next implementation round)

**Date:** 2026-08-14
**Purpose:** Carry context into a fresh session to implement the high-level
"go to a goal" action tier on top of the (already-shipped) trajectory-execution
frontend. Read this first, then confirm the open items, then brainstorm → spec →
plan → build (subagent-driven).

---

## 1. Where things stand (all DONE + MERGED + real-arm validated)

Three plans shipped this session; the full stack was validated on the physical
Kinova Gen3 arm today (single-joint and coordinated two-joint
`ExecuteJointTrajectory` moves, both `SUCCEEDED` on target).

- **Core driver** — `rammp-org/kinova-gen3-driver` @ `main` (Plan 1 + Plan 2, PRs #9/#11).
  - Layer A ports (`Action`/`Stream`/`Service` + value types), Layer C `Supervisor`
    (sampler + state pump threads, mode-at-rest switching, full
    preempt/cancel/gapless-queue result accounting), b1 install/export
    (`find_package(kinova_lowlevel CONFIG)`), 1 kHz RT execution core.
  - Key seam: **`kinova::interface::CommandSink`** (`include/kinova_lowlevel/interface/ports.h`)
    — `on_trajectory_goal / on_trajectory_accepted / on_trajectory_cancel /
    on_set_gains / on_query_state`. The `Supervisor` implements it. Everything
    that commands motion funnels through here.
- **ROS2 frontend** — `rammp-org/kinova_gen3_ros2` @ `main` (Plan 3, PR #1).
  - `kinova_gen3_interfaces` (ament): `ExecuteJointTrajectory.action` +
    `JointImpedanceGains.msg`.
  - `kinova_gen3_ros2` (ament): `Ros2Backend` (the ONLY unit that includes rclcpp —
    action server + driven ports + `/joint_states` publisher), pure
    value-type↔message `message_mapping`, and the DI **bring-up node**
    `src/bringup_node.cpp` → executable `kinova_gen3_node`.
  - `.repos` pins the core to `main`.

Design records: `docs/superpowers/specs/2026-08-12-ros2-backend-realization-design.md`,
`docs/on-robot-runbook.md`, and the core repo's
`docs/superpowers/specs/2026-08-10-arm-driver-interface-design.md` +
`2026-08-12-ros2-build-integration-strategy.md`.

---

## 2. The decision for this round (LOCKED in discussion)

Build a **high-level, goal-oriented action tier**: `GoToEEPose`,
`GoToJointConfig`, `GoToPreset`. Decisions already made:

1. **EE pose (and any collision-aware goal) goes through cuRobo.** The driver does
   NOT get a Cartesian/IK command mode for this. Pre-planned joint trajectories
   still execute directly via `ExecuteJointTrajectory` (full planner bypass).
2. **cuRobo is a separate, already-existing ROS node.** It takes a target pose and
   generates a trajectory. We do NOT build cuRobo — we call it over ROS.
3. **ONE node of ours hosts everything.** All action servers
   (`ExecuteJointTrajectory` **and** the high-level ones) live in the single
   existing `kinova_gen3_node`, which is ALSO a **client** of the cuRobo node.
   We do NOT add a separate orchestration node.
4. **High-level actions feed the planned trajectory into the same `Supervisor`
   seam internally — no self-call of the `ExecuteJointTrajectory` action over ROS.**
   A `GoToEEPose` handler: call cuRobo (ROS) → get `JointTrajectory` → build a
   `kinova::interface::TrajectoryGoal` → hand it to the `Supervisor`'s
   `CommandSink` (same path `Ros2Backend`'s `ExecuteJointTrajectory` handler uses).

System picture (two nodes total: ours + the external cuRobo node):

```
   GoToEEPose / ExecuteJointTrajectory
[client] ───────────────────────────▶ ┌─ kinova_gen3_node (ONE node, ours) ──────┐
                                       │  • all action servers                    │
                                       │  • Supervisor + 1 kHz RT loop            │
                                       │  • cuRobo client ────────────────────────┼──▶ [ cuRobo node ] (separate, exists)
                                       └──────────────────────────────────────────┘
```

Pre-planned `ExecuteJointTrajectory` skips cuRobo. A `GoToEEPose` goal:
client → our server → cuRobo (plan) → trajectory → our `Supervisor` → execute →
result to client.

---

## 3. Refinements / hazards to honor in the design

- **Action-vs-action arbitration is already handled.** Because every server funnels
  into ONE `Supervisor`, its single-active-commander + latest-wins/queue logic IS
  the arbitration for actions. (Cross-*tier* arbitration — a future streaming
  topic vs actions — is a separate spec, not this round.)
- **Do NOT block the rclcpp executor on the cuRobo call.** Planning can take tens
  to hundreds of ms or stall. A synchronous wait on the default single-threaded
  executor starves the `ExecuteJointTrajectory` server + feedback. Use an **async**
  action/service client to cuRobo + a **reentrant/separate callback group** (or a
  multithreaded executor). (`/joint_states` pump is already its own thread.)
- **Keep the high-level handlers THIN** (translate → call cuRobo → feed Supervisor).
  Watch god-node drift: if task logic/perception/multiple planners accrete, that's
  the signal to lift these into their own node later — cheap to do *if* handlers
  stay thin and already go through the `Supervisor` seam.
- **Cancel must propagate:** cancel of a high-level goal → cancel the underlying
  cuRobo plan request AND/OR the in-flight `Supervisor` trajectory.
- **Planning failure is distinct from execution failure:** e.g. a `PLANNING_FAILED`
  result code separate from `PATH_TOLERANCE_VIOLATED`/execution errors.

---

## 4. OPEN — resolve FIRST in the next session (blocks the action contracts)

1. **cuRobo's ROS interface (the critical unknown; user was "figuring out the
   interface to cuRobo").** Need: action vs service; exact message types; does it
   take just a target pose or pose + start state (does it read `/joint_states`
   itself or expect the start config in the request?); does it return a
   `trajectory_msgs/JointTrajectory`; does it own/collision-check a world? These
   shape the goal/feedback/result of `GoToEEPose` and how our node calls it.
2. **High-level action message shapes** — new `.action` defs in
   `kinova_gen3_interfaces` (`GoToEEPose.action`, `GoToJointConfig.action`,
   `GoToPreset.action`), including the `PLANNING_FAILED` result code + feedback
   (relay cuRobo planning progress, then execution progress?).
3. **`GoToPreset`:** does the driver need a named-preset registry (config/param),
   or does the client supply the config? (A joint-config preset is a degenerate
   `ExecuteJointTrajectory` today — the value-add is the named registry.)
4. **Threading concretely:** callback-group layout so cuRobo planning + trajectory
   execution + feedback all stay responsive.

---

## 5. Deferred (NOT this round — future specs)

- **Streaming / Topic tier** (streaming EE pose/vel/torque, joint pos/vel/torque
  for reactive closed-loop). Highest RT-safety bar. Before building it, write the
  **safety contract**: staleness watchdog (setpoints stop → hold or ramp-to-stop,
  never a stale/zero command), rate limits, "reuse-last-good, never
  default-on-failure" (see the bug in §6). EE modes are three DISTINCT Jacobian
  laws (pose→IK, vel→J⁺, torque→Jᵀ) = distinct `ControlMode`s.
- **Cross-tier arbitration / control-ownership** (acquire/release, safe handoff) —
  its own spec. Hooks exist (`sender_id` + the single admit seam in
  `on_trajectory_goal`).
- **`set_gains` / `query_state` services** (stubbed in the Supervisor) and a
  **`JointState` diagnostic topic** (loop-timing) — ROS2 fast-follow.

---

## 6. Hard-won operational facts (don't relearn these)

- **Builds are aarch64-only on `abra`** (muk cannot build). Deploy loops (rsync
  muk→abra + colcon):
  - ROS2 workspace: `scripts/abra_colcon.sh [colcon args]` (rsyncs the core in via
    the deploy script; `.repos` documents the git source). Sim e2e:
    `scripts/abra_e2e_sim.sh`.
  - Core unit tests: the core repo's scratch `abra_test.sh '<gtest filter>'`
    (rsync muk→`abra:/tmp/kinova-build`, sim build + ctest) — recreate the scratch
    script next session (path is session-specific).
- **Running the node on abra over ssh MUST use `ssh abra 'bash -lc "source
  /opt/ros/humble/setup.bash; ..."'`** — abra's default shell is zsh and chokes on
  ROS's bash `setup.bash` (`ros2: command not found`).
- **`/joint_states` is best-effort (`SensorDataQoS`)** — subscribers/echo must use
  best-effort QoS or they receive nothing.
- **`ros2 run` forks the node under a python wrapper**, so `kill %1`/`$!` misses the
  real process → use **`pkill -TERM -f kinova_gen3_node`** and always verify it
  stopped (abra is shared; a leaked node keeps servoing the arm).
- **Combined KORTEX+ROS2 build:** `bash scripts/abra_colcon.sh --cmake-args
  -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64`.
  The core's static-lib PRIVATE KORTEX link auto-propagates into the export → node
  links KORTEX with NO core change. **The CMake cache PERSISTS the flag across
  rebuilds — always pass `-DKINOVA_ENABLE_KORTEX=ON/OFF` explicitly.**
- **Test client** `kinova_gen3_ros2/test/send_trajectory.py`: seeds waypoint-0 from
  the measured `/joint_states` pose (safe local move, never jump-to-zero);
  multi-joint via comma-lists (`--joint 5,6 --delta 0.4,-0.4`); **put a positive
  `--delta` first** (argparse treats a leading `-` value as a flag). The action
  feedback's `actual` field IS the live measured q → use it to confirm real motion.
- **Attended real-arm procedure:** `docs/on-robot-runbook.md`. Dry-run/read-only
  first (confirm `/joint_states` shows the REAL pose); small/slow/distal; e-stop in
  hand; never connect unattended.
- **THE BUG (fixed, core `dd3e57e`), keep the lesson:** the Supervisor sampler reset
  `q_meas` to `Zero()` each tick and only overwrote on a successful lock-free
  `Seqlock` read → a rare failed read (RT writer preempted mid-store) injected
  `q=0` into the divergence guard → FALSE `PATH_TOLERANCE_VIOLATED` abort
  mid-motion on the arm (sim never hit it). Fix: persist last-good `q_meas`, reuse
  on failed read (`sampled_q` in `supervisor.h`). **Lesson for the streaming tier:
  any lock-free-read consumer must reuse-last-good, never inject a default on read
  failure.**

---

## 7. Process for the round

Subagent-driven, same loop that just worked: brainstorm (lock the §4 open items)
→ writing-plans → subagent per task with a per-task review + a whole-branch review,
the controller running the abra build/test loop (subagents CAN ssh to abra). The
new `.action` defs go in `kinova_gen3_interfaces`; the servers + cuRobo client go in
`kinova_gen3_ros2` (the existing node). Keep the core untouched unless a new bug
demands it.

Memory notes to rely on: `arm-interface-layer`, `kinova-gen3-ros2-repo`
(both updated 2026-08-14 with status + the operational facts above).
