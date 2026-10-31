# Changelog

## Unreleased

## 2.0.0 — 2026-10-30

A major release. The arm is described in its own mounted frame (`arm_mount`, every
name prefixed `arm_`), so tilted mounts plan and move correctly. The node survives arm
faults and disconnects instead of exiting, stops on unexpected contact, runs compliant
(impedance) motion on every command path, and plans through several targets in one
motion.

Built against **kinova-gen3-driver 2.0.0**, **RAMMP-CuRobo 2.0.0** and
**rammp-interfaces-ros2 2.0.0** (via `rammp-base:2.0.0-jp6`). Move all four together,
on every robot, dojo included. With RAMMP-CuRobo 1.1.0 or earlier, every GoTo goal
settles `PLANNING_FAILED`. Clients must rebuild against rammp-interfaces-ros2 2.0.0.

### Upgrading from 1.1.0

**Names and frames.**

- Every arm joint and link name now starts with `arm_`, and the model's root is
  `arm_mount` (Z against gravity when the chair is level). `/joint_states` carries
  `arm_joint_1`…`arm_joint_7` and `arm_robotiq_85_left_knuckle_joint`, taken from the
  model. There is a new frame `arm_tool_frame`, cuRobo's `tool_frame`, 0.120 m beyond
  the flange (#47).
  *Do:* rename `joint_N` → `arm_joint_N` and `base_link` → `arm_mount` in clients,
  configs and saved data.
- `GoToEEPose` requires `frame_id: arm_mount`. `base_link` is rejected, and the node
  log gives the reason. On flatbot and gen_1_5, Cartesian numbers change by the 30°
  mount (intended) (#47).
  *Do:* re-record saved poses on tilted robots.
- Trajectory `joint_names` are checked, not ignored (#68, closes #11). An
  `execute_joint_trajectory` goal whose `joint_names` are not exactly
  `arm_joint_1`…`arm_joint_7`, in order, is rejected, and so is one with no names. A
  non-empty `path_tolerance` or `goal_tolerance` must name the same joints in the same
  order. A planned move whose cuRobo trajectory names other joints, or orders them
  differently, settles `PLANNING_FAILED` and never executes.
  *Do:* update clients that still send `joint_1`…`joint_7`: arm-examples,
  RAMMP-generalized's `make_ros_goal`, microwave-playground.
- On a tilted mount (more than 0.5° from level), `GoToEEPose` refuses a plan whose
  `header.frame_id` is not `arm_mount`, rather than moving the arm off by the tilt
  (#47). RAMMP-CuRobo 2.0.0 plans in `arm_mount` and stamps the frame.

**Launch and model.**

- The arm's model is rendered at launch from the robot's `mount.yaml` and published,
  latched, on `/arm/robot_description` by `arm_state_publisher` (#47).
  `description.launch.py` and `bringup.launch.py` require `mount_file`, with no
  default. The build no longer produces `kinova_gen3.urdf` or `kinova_gen3_7dof.urdf`,
  and `expand_model.py` is gone. `kinova_gen3_render.py --mount --out` renders a model
  for the bench.
  *Do:* give every robot a `mount.yaml` and run `arm_state_publisher` alongside the
  driver (see `docs/getting-started.mdx`).
- `kinova_gen3_node --urdf` is removed. The node reads `/arm/robot_description` before
  the RT loop starts. It waits 30 s, then fails with an error naming
  `arm_state_publisher`. `--ee-frame` defaults to `arm_end_effector_link`. The node logs
  the model's mount tilt at startup. If a different model is published later, it logs
  an ERROR and keeps the startup model until restarted (#47).
  *Do:* drop `--urdf` from launch commands.
- The driver's model is the upstream Kinova composition, replacing the hand-edited
  `models/gen3_7dof_2f85.urdf` (#47):
  - The bracelet is the no-vision one (0.364 kg, was the 0.500 kg vision variant); the
    xacro's `camera` arg defaults to `false`.
  - The wrist is 0.572 kg lighter (no D415 or mount). *Do:* put a real wrist camera's
    mass in the model.
  - Joint limits on j2/j4/j6 tighten to Kinova's ±2.24/±2.57/±2.09 rad (were
    ±2.41/±2.66/±2.23). `test_model_parity.py` pins both.

**Behaviour.**

- `/joint_states` `effort` flips sign. It is now the torque the actuator applies, ≈
  `+g(q)` at a free hold, instead of KORTEX's raw reaction sign. The driver normalizes
  it at its transport boundary (kinova-gen3-driver 2.0.0).
  *Do:* check any consumer that compares effort to a gravity model.
- `/joint_states` and `/ee_state` publish at ~50 Hz, down from ~100 Hz. Each pump tick
  is decimated by the new `state_publish_divisor` (default 2). Both topics stay on the
  same ticks (#46).
- A `QUEUE` trajectory goal sent while another goal is running is refused: it settles
  `INVALID_GOAL`, and the running goal carries on. On an idle arm, `QUEUE` runs as
  before (`preemption = 0` still maps to it). The driver dropped queueing because it
  was unsafe as built.
  *Do:* plan one goal through all the waypoints, or use `go_to_ee_poses` /
  `go_to_joint_configs` (kinova-gen3-driver#84).
- `ExecuteJointTrajectory`'s `gains` is now an `ImpedanceGains` (retyped in
  rammp-interfaces-ros2 2.0.0) (#44).
- The node no longer exits on an arm fault or disconnect. It waits for `/clear_faults`
  (see Added). Supervisors that restarted the container on exit no longer see one.

### Added

**Compliant (impedance) execution everywhere** (#44, kinova-gen3-driver#63).

- Every command path accepts an `ImpedanceGains`: a named profile (soft / medium /
  stiff), custom gains, or the session default. That covers trajectory and GoTo goals
  and stream opens.
- `/set_gains` (`rammp_arm_interfaces/srv/SetGains`, token-gated) sets the session
  default. It touches no live mode.
- GoTo goals run under impedance use a relaxed divergence guard: 0.60 rad, against
  0.35 rad for position.
- Two new stream controllers, `joint_velocity_impedance` and `ee_twist_impedance`, the
  compliant velocity and twist tiers.

**Fault survival** (#63, kinova-gen3-driver#96, #59, #100).

- The node runs the driver's `ArmLink`. An arm that is off or unreachable is retried
  every second, forever, including one still booting.
- A fault hands the arm back to its base and waits for `/clear_faults`
  (`std_srvs/Trigger`, new). Faults include e-stop, protective stop, the arm taken out
  of low-level, a rejected command, or the link lost while running.
- While the arm is unavailable, in-flight goals settle `HALTED`, and new goals and
  streams are refused.
- New `Arm link` task on `/diagnostics`. The `Arm` task goes STALE when feedback is
  over 1 s old.
- Only a robot with the wrong joint count still exits (FATAL, exit 1).

**Contact detection** (#67, kinova-gen3-driver#109, closes the code side of #24).

- An unexpected contact stops the arm and floats it. A goal running at the trip ends
  `-11 CONTACT_STOP`, distinct from `-9 HALTED`; so does a GoTo goal still planning.
- While latched, every motion action, stream open and gripper setpoint is refused with
  the reason. `/clear_faults` clears a contact stop as well as a fault, in one call.
- `contact_detection`: `enforce` (default), `observe` or `off`. Observer gain,
  debounce, threshold profiles, the threshold ceiling and float settings are read-only
  launch parameters. A bad value fails startup, naming the parameter and the joint.
- `/set_contact_thresholds` (token-gated) switches the profile, or sets exact per-joint
  values for live tuning. Nothing is clamped: an out-of-range request is refused.
  Default thresholds are `[16,16,12,12,4,4,4]` N·m, sensitive `[8,8,6,6,2,2,2]`, loose
  `[24,24,18,18,6,6,6]`, ceiling `[30,30,22,22,7,7,7]`.
- New `Contact detection` task on `/diagnostics`: mode, phase, profile, residuals, and
  the last trip.
- `/contact_monitor/residual` (only with `contact_debug_topic:=true`) publishes the
  per-joint residual at 50 Hz, for tuning.
- `--sim` installs the sim gravity model, so the sim does not false-trip.
- Runbook: `docs/on-robot-runbook.md`; contract: `docs/interface.md` "Contact stops".

**Motion.**

- `go_to_ee_poses` and `go_to_joint_configs` move through a list of targets in one
  motion (#65, rammp-interfaces-ros2#21).
  - Each target (`EEPoseTarget` / `JointConfigTarget`) carries its own leg's
    `speed_scale`, `orientation_hold` (poses only) and `blend_radius`.
  - Each leg is planned from the last point of the one before. RAMMP-CuRobo's
    `/rammp_curobo/blend` joins the legs, and the result runs once.
  - A planning failure ends the goal `PLANNING_FAILED` before anything moves, as
    `leg k/N: <reason>` or `blend: <reason>`. A cancel stops the chain before the next
    leg.
  - Known limit: another GoTo goal sent mid-chain can collide with it in the planner
    (`planner busy`).
- `/ee_state` carries the external EE wrench (#46). While contact detection runs and
  its observer has settled, the wrench is the momentum observer's estimate, which
  stays accurate in motion and trails by about 25 ms. Otherwise it is the quasi-static
  `g(q) − τ` estimate: with detection off, for ~200 ms after each start or clear, and
  while the link waits for `/clear_faults` (#70, kinova-gen3-driver#112).
- `home` and `rest` are required preset names, and every node has both (#50,
  kinova-gen3-driver#86). They are registered even when `preset_names` omits them.
  `home` defaults to Kinova's factory Home, `rest` to Kinova's Retract with joint 4
  at -2.356 rad (#55). Installations override the angles in config. There is no `zero`
  preset.

**Arbitration.**

- `acquire_control` takes an optional pre-generated `token` (#58,
  kinova-gen3-driver#89), so an orchestrator can share one key across several
  resources. Left all-zero (the default), the driver mints one as before.

**Real-time.**

- Bringup creates the Kortex SDK's UDP receive thread at SCHED_FIFO `--rt-priority`−1
  on the RT core (`--cpu`). The TCP receive and keepalive threads stay normal (#62,
  kinova-gen3-driver#98, rammp-deployments#120). On flatbot under the full perception
  load, overruns over 30 min fell from 66–148 to 9, with no comm timeouts.

**Build and CI.**

- The required `smoke` CI leg builds the composition a PR proposes: the same-named
  companion branch in the driver and interfaces repos, else `dev`. It does this through
  `CORE_REF` and the new `INTERFACES_REF` Docker build arg. The released-pins build
  survives as the informational `smoke-released` (#45).
- The image contains `kinova_gen3_description`, so `arm_state_publisher` runs from it.
  The 1.1.0 image lacked it, although the deployment manifest already ran
  `description.launch.py` from it (#47).

### Fixed

- `/tf` is published on every `/joint_states`, not throttled to robot_state_publisher's
  default 20 Hz. `description.launch.py` sets `publish_frequency` to 1000 (#61). Before,
  /tf came at 16.4 Hz with 61 ms gaps, so half of the wrist camera's frames waited for a
  future transform.
- `GoToPreset rest` can be planned to. The factory Retract self-collides in cuRobo's
  sphere model (the gripper overlaps the shoulder link by 27 mm), and every plan from
  home failed `IK_FAIL`. Joint 4 opens from -2.548 to -2.356 rad (#55).
- `Ros2Backend::settle` logs a refused terminal goal transition instead of letting the
  exception terminate the node with the arm powered (#64, closes #7).
- Test-only: the contact tests stop their executor safely, with no lost-cancel hang or
  teardown segfault (#71, closes #69).
- Docs: the GoToEEPose guide link in the planned-moves guide (#52). The README now
  covers queue refusal (#66), and the README and `docs/interface.md` the observer wrench (#70).

## 1.1.0 — 2026-10-02

> **RESOLVED 2026-10-02: the three dependency releases exist and the pins have
> moved.** Until then this branch did not build from its own declared sources —
> a clean `vcs import` + `docker build` failed to compile `message_mapping.cpp`
> and `curobo_plan_client.cpp`, and verification ran via a hand-mounted
> four-source colcon workspace on the Jetson. The record, with the pins as they
> were:
>
> | Dependency                                           | Was pinned                                                            | Now pinned                        | Release                                   |
> | ---------------------------------------------------- | --------------------------------------------------------------------- | --------------------------------- | ----------------------------------------- |
> | `kinova-gen3-driver` (`kinova_gen3.repos`)           | `v1.1.1` (no `TrajectoryGoal::speed_scale`, no `kMinSpeedScale`)      | `v1.2.0`                          | `speed_scale` + runtime override          |
> | `RAMMP-CuRobo` (`kinova_gen3.repos`)                 | `v1.0.0` (`PlanToPose.action` lacks `hold`/`approach_via`)            | `v1.1.0`                          | pose constraints + via point              |
> | `rammp-interfaces-ros2` (base image in `Dockerfile`) | v1.0.0 via `rammp-base:1.0.0-jp6` (an in-tree 1.1.0 was never tagged) | v1.1.0 via `rammp-base:1.1.0-jp6` | `speed_scale` + `orientation_hold` fields |

> **`approach_offset` has been REMOVED from the arm interface, not shipped.**
> It was plumbed end to end and provably reached cuRobo (an oversized offset
> made a plan infeasible, which an ignored field could not do), but what it did
> to a trajectory was never understood and on-arm runs produced motion the
> operator described as "very strange". Everything measured is in #40, including
> the two documented claims about it that turned out false. The planner keeps
> the capability; nothing drives it.

### Added

- `GoToEEPose` accepts `orientation_hold` (`HOLD_NONE`, `HOLD_LEVEL` or
  `HOLD_FIXED`, constants on the goal) and `speed_scale`. `GoToJointConfig`,
  `GoToPreset` and `ExecuteJointTrajectory` accept `speed_scale` only. Both
  fields default to off, so existing goals behave as before.
- An unknown hold mode is **refused**, in the server and again in
  `CuroboPlanClient`, rather than treated as `HOLD_NONE`. Running an
  unconstrained move for a caller who asked for a held one would look like
  success, which is the worst way for this to fail.
- A hold the planner cannot satisfy — a goal orientation that disagrees with
  where the arm is — settles `PLANNING_FAILED (-7)` with the measured deviation
  in `error_string`, refused before planning rather than silently re-aimed.
- Superseded before release, never published: an earlier cut carried
  `ToolAxisLock` (six per-axis booleans plus a reference frame) and
  `ApproachOffset`. Per-axis holds expressed a mechanism rather than a request;
  position holds could only ever mean "travel along one base axis", and the
  general case is not expressible through the planner's diagonal constraint
  weighting at all; and the frame field could be set and silently disregarded.
- `speed_scale` is passed to the driver's `TrajectoryGoal`, which runs the
  trajectory slower by dilating its executor clock. An out-of-range value is
  refused with a reason in the node log, not clamped.
- Interface change: fields appended at the end of existing messages with
  defaults, plus two new messages. A **minor** bump of `rammp-interfaces-ros2`
  (released as 1.1.0 — the earlier in-tree 1.1.0 was never tagged), under
  Cyclone DDS.

### Known limitations

- A refused goal returns a bare action rejection with no payload; the reason is
  logged server-side only (`kinova-gen3-ros2#39`).

### Fixed

- `kinova_gen3.repos` stated the interface versioning rule backwards; it now
  points at the authoritative policy in `rammp-interfaces-ros2`.
