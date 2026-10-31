# Changelog

## Unreleased

### Added

- **`/ee_state` wrench from the contact observer, and `wrench_source`**
  (rammp-org/kinova-gen3-driver#112). While contact detection runs (`enforce` or
  `observe`) and its observer has settled, the wrench is the momentum observer's
  estimate, which stays accurate while the arm moves and lags by about 25 ms.
  Otherwise (`contact_detection:=off`, the ~200 ms settle after each start and each
  contact clear, a link waiting for `/clear_faults`) it is the quasi-static estimate
  as before. New
  `EeState.wrench_source` (`WRENCH_SOURCE_QUASI_STATIC` / `WRENCH_SOURCE_OBSERVER`)
  says which. Needs the matching driver and `rammp_arm_interfaces` (built with them
  via `CORE_REF` / `INTERFACES_REF` until their releases).
- **`go_to_ee_poses` and `go_to_joint_configs`**: `GoToEEPose` / `GoToJointConfig`
  through a list of targets in ONE motion (rammp-interfaces-ros2#21). Each target
  (`EEPoseTarget` / `JointConfigTarget`) carries its own leg's `speed_scale`,
  `orientation_hold` (poses) and `blend_radius`. Each leg is planned from the last
  point of the one before, the legs are blended by cuRobo's new
  `/rammp_curobo/blend` (RAMMP-CuRobo#48), and the result is executed once, at speed
  scale 1 because it carries the per-leg speeds. A failure while planning ends the
  goal `PLANNING_FAILED` before anything moves, as `leg k/N: <reason>` (k from 1) or
  `blend: <reason>`; `planner_state` carries the same prefix. A cancel stops the
  chain before the next leg. Known limit: another GoTo goal sent mid-chain can
  collide with it in the planner (`planner busy`). `kinova_gen3.repos` pins
  RAMMP-CuRobo at #47's commit until its release.
- **The node no longer exits on arm-side events** (driver #96, #59, #100). It runs
  core's `ArmLink`: an arm that is off or unreachable is retried every second,
  forever, including an arm still booting; a fault (e-stop, protective stop, the arm
  taken out of low-level, a rejected command, or the link lost while running) hands the arm back to its
  base and waits for **`/clear_faults`** (`std_srvs/Trigger`, new). New `Arm link`
  task on `/diagnostics`. While the arm is unavailable, in-flight goals settle HALTED
  and new goals and streams are refused. Only a robot with the wrong joint count still
  exits (FATAL, exit 1). The `Arm` task now goes STALE when feedback is over 1 s old.
  Needs the driver's `feat/96-never-crash` (built with it via `CORE_REF` until a
  driver release).

### Changed (breaking)

- **Trajectory `joint_names` are checked, not ignored** (#11). Positions and
  tolerances are mapped by index, so an `execute_joint_trajectory` goal whose
  `joint_names` are not exactly `arm_joint_1`…`arm_joint_7` in order is rejected,
  and so is one with no names. A non-empty `path_tolerance` or `goal_tolerance` must
  name the same joints in the same order, so unnamed tolerances are refused too. A
  planned move whose cuRobo trajectory names other joints, or the same joints in another
  order, settles `PLANNING_FAILED` and is never executed. **Every GoTo action now needs
  RAMMP-CuRobo with #38**, whose plans carry `arm_joint_1`…`arm_joint_7`; with
  RAMMP-CuRobo v1.1.0 or earlier, which carry `joint_1`…`joint_7`, every GoTo goal
  settles `PLANNING_FAILED`. Clients sending `joint_1`…`joint_7` (arm-examples,
  RAMMP-generalized's `make_ros_goal`, microwave-playground) are now refused.
- **The arm's model is rendered at launch from the robot's `mount.yaml` and published
  on `/arm/robot_description`** (latched). `description.launch.py` and
  `bringup.launch.py` require `mount_file` and have no default. The build no longer
  produces `kinova_gen3.urdf` or `kinova_gen3_7dof.urdf`, and `expand_model.py` is gone.
  `kinova_gen3_render.py --mount --out` renders one for the bench.
- **Every arm name carries `arm_`, and the root is `arm_mount`** (Z against gravity when
  the chair is level). `/joint_states` carries `arm_joint_1`…`arm_joint_7` and
  `arm_robotiq_85_left_knuckle_joint`, taken from the model. New frame
  `arm_tool_frame`: cuRobo's `tool_frame`, 0.120 m beyond the flange.
- **`kinova_gen3_node --urdf` is removed.** The node reads `/arm/robot_description`
  before the RT loop starts (30 s timeout, error names `arm_state_publisher`). Core
  reduces the model to the arm. `--ee-frame` defaults to `arm_end_effector_link`.
- **`GoToEEPose` requires `frame_id: arm_mount`.** `base_link` is rejected, and the
  reason is in the node log. On flatbot and gen_1_5, Cartesian numbers change by the
  30° mount (intended). Re-record saved poses there.
- **The model's bracelet is the no-vision one** (0.364 kg, was the 0.500 kg vision
  variant). The xacro's `camera` arg now defaults to `false`; RAMMP's arms have no
  Kinova vision module. On abra, gravity comp with it holds joints 2 and 4 at 0.00 °/s.
- **The driver's model is the upstream composition**, replacing the hand-edited
  `models/gen3_7dof_2f85.urdf`. The wrist is **0.572 kg
  lighter** (no D415 + mount: put a real wrist camera's mass in the model), and the
  **j2/j4/j6 position limits tighten to Kinova's ±2.24/±2.57/±2.09 rad** (were
  ±2.41/±2.66/±2.23). Today's cuRobo still plans with the old limits, so a plan through
  the difference can abort with `PATH_TOLERANCE_VIOLATED` until cuRobo reads this model.
  `test_model_parity.py` pins both.
- **On a tilted mount, `GoToEEPose` refuses a plan that is not in `arm_mount`.** The
  planner stamps the frame it planned in on the trajectory's `header.frame_id`. When the
  model's mount is more than 0.5° from level and that frame is not `arm_mount`, the goal
  settles `PLANNING_FAILED` naming it, instead of moving the arm by the tilt. Today's
  cuRobo plans in its own arm base and leaves the frame unset, so on flatbot pose goals
  are refused until RAMMP-CuRobo plans in `arm_mount` and stamps it. Level mounts (dojo)
  and joint-space goals are unaffected by this check, but #11 (above) makes every GoTo
  goal need that RAMMP-CuRobo.
- The node logs the model it uses at startup (`model from /arm/robot_description: mount tilt N deg ...`, read from the model), and an ERROR whenever a different model is
  published on the topic later; it keeps the startup one until restarted.
- `/arm/robot_description` is a required subscription in the driver's fragment and in
  `docs/interface.md`.
- The image contains `kinova_gen3_description`, so `arm_state_publisher` runs from it.
  The 1.1.0 image lacked it, although the deployment manifest already ran
  `description.launch.py` from it.
- **A `QUEUE` trajectory goal sent while another goal is running is refused**
  (driver #84): it settles `INVALID_GOAL` and the running goal carries on. On an
  idle arm `QUEUE` runs as before. The driver dropped queueing because it was unsafe
  as built (no check that a queued goal starts where its predecessor ends, goals
  complete on time rather than on arrival); plan one goal through all the waypoints
  instead. No code change here (`preemption = 0` still maps to `kQueue`); needs
  driver rammp-org/kinova-gen3-driver#106 until a driver release.

### Added

- **`home` and `rest` are the required preset names, and every node has both**
  (rammp-org/kinova-gen3-driver#86). Callers code against the names. Both are registered
  even when `preset_names` omits them. Defaults: `home` is Kinova's factory Home;
  `rest` is Kinova's Retract with joint 4 at -2.356 rad (factory -2.548 self-collides
  in cuRobo).
  An installation overrides the angles in config, with no release. There is no `zero`
  preset.
- **`acquire_control` takes an optional pre-generated `token`**
  (rammp-org/kinova-gen3-driver#89), so an orchestrator can use one key across several
  resources. Leave it all-zero (the default) and the driver mints one as before.
  Needs the matching `rammp_common_interfaces` (`AcquireControl` request gains the
  field) and a core with `grant(owner_id, token)`.

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
