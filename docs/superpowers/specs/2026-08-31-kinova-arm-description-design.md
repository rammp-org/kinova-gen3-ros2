# `kinova_gen3_description` — a regenerable robot model, TF, and launch (design)

**Date:** 2026-08-31
**Status:** Design (approved by user; ready to plan/implement)
**Repo:** `rammp-org/kinova_gen3_ros2`
**Core dependency:** none new. Relies on core PR #33, already open, for the
`package://` mesh rewrite.

## What this is

A sister package holding the robot model, plus the launch plumbing that makes the arm
legible to an ordinary ROS user: `/robot_description`, TF via `robot_state_publisher`,
and a bringup launch that starts the driver alongside it.

It also fixes two defects that make the current model unusable outside this repo.

## The two defects

**1. TF does not work today, and silently.** The driver publishes `joint_1 … joint_7`
on `/joint_states`; the URDF declares `gen3_joint_1 … gen3_joint_7`. `robot_state_publisher`
matches by name, so with zero overlap it would publish TF for the **default
configuration and never update it** — the arm sits frozen in RViz while really moving.
Nothing errors.

**2. The model is not regenerable.** `models/gen3_7dof_2f85.urdf` is a hand-edited
artifact from `c3pzero_ws`, a Clearpath mobile-base workspace we do not have. Its 37
mesh paths pointed at absolute paths inside that container (fixed in core PR #33), its
joints carry a `gen3_` prefix that exists only to avoid collisions with a mobile base we
do not use, and **every Robotiq joint is `type="fixed"`** — someone froze the gripper, so
it cannot articulate at all.

The mesh paths and the naming are symptoms. The disease is that nobody can reproduce the
file or say how it was made.

## Decisions (user-approved)

1. **Generate the model from upstream xacros** — `kortex_description` +
   `robotiq_description` — rather than vendoring the hand-edited URDF. Provenance becomes
   a build step; the arm configuration becomes a set of xacro args.
2. **Drop the `gen3_` prefix.** Joints become `joint_1 … joint_7`, matching what the
   driver already publishes and what every other Kinova stack uses.
3. **Two launch files**: a description launch (reusable by anyone who wants only TF) and
   a bringup launch that includes it and starts the driver.
4. **Gripper joint state:** publish the single actuated joint; let `<mimic>` do the rest.
5. **Gripper velocity and effort are NaN for now**, and become real once core reads them.

## Package layout

```
kinova_gen3_description/
  package.xml            exec_depends: kortex_description, robotiq_description,
                         realsense2_description, robot_state_publisher, xacro
  CMakeLists.txt         ament_cmake; installs urdf/ and launch/, and expands
                         the xacro to a plain .urdf at build time
  urdf/
    kinova_gen3.urdf.xacro       the composition; args: gripper, camera, prefix
  launch/
    description.launch.py       /robot_description + robot_state_publisher
    bringup.launch.py           includes description, starts kinova_gen3_node
```

No RViz config: the approved scope is the two launch files. `rviz2` against
`/robot_description` works without one, and a checked-in config is another artifact to
keep current as the model changes. Easy to add later if the default view is annoying.

The three upstream description packages are already installed on abra and are
apt-installable (`ros-humble-kortex-description` and friends), so depending on them
costs three lines and no vendoring. **No meshes are copied into this repo.**

## The model

`kinova_gen3.urdf.xacro` composes the upstream macros:

```xml
<xacro:arg name="gripper" default="robotiq_2f_85"/>   <!-- or "none" -->
<xacro:arg name="camera"  default="true"/>            <!-- wrist realsense -->
<xacro:arg name="prefix"  default=""/>                <!-- empty by default -->
```

The two current URDFs (`gen3_7dof.urdf`, `gen3_7dof_2f85.urdf`) collapse into
`gripper:=none` and `gripper:=robotiq_2f_85`. `prefix` exists because upstream offers it
and a future mobile base would need it — it is **not** used by default, and the `gen3_`
prefix does not come back.

### Core still needs a plain URDF, and that is fine

Core's `Dynamics` loads a URDF **path** at runtime (`--urdf`), and its tests compile
against a `URDF_PATH` define. Xacro is a ROS build-time tool; Pinocchio will not expand
it. So:

- This package installs **both** the `.xacro` and a **pre-expanded `.urdf`** generated at
  build time by `xacro` into the install space.
- Launch files use the xacro (standard ROS practice, allows args).
- Core's `--urdf` points at the expanded file in this package's share directory.
- **Core keeps its own `models/` copy** for its standalone apps and unit tests, which must
  build with no ROS at all. That is a deliberate duplication, bounded by the parity check
  below.

### The `ee_frame` coupling

`Dynamics`'s constructor defaults `ee_frame = "gen3_end_effector_link"` — a **link** name
resolved by string. Dropping the prefix renames that link to `end_effector_link`, so the
default must change with it, and any call site passing the old name explicitly must be
audited. Getting this wrong does not fail loudly: `fk()` would resolve a different frame
and every Cartesian surface — `/ee_state`, `/go_to_ee_pose`, EE-pose streaming — would be
silently offset.

This is the single most dangerous edit in the plan and gets its own verification step.

## Gripper joint state

The upstream 2F-85 macro models the gripper as **one actuated joint and five mimics**:

| joint | mimic of `robotiq_85_left_knuckle_joint` |
|---|---|
| `robotiq_85_left_knuckle_joint` | — *revolute, limits `[0.0, 0.8]` rad, the only real DOF* |
| `robotiq_85_right_knuckle_joint` | ×−1 |
| `robotiq_85_left_inner_knuckle_joint` | ×1 |
| `robotiq_85_right_inner_knuckle_joint` | ×−1 |
| `robotiq_85_left_finger_tip_joint` | ×−1 |
| `robotiq_85_right_finger_tip_joint` | ×1 |

**Publish only the actuated joint.** `robot_state_publisher` reads the `<mimic>` tags and
derives the other five itself (its binary carries `urdf::JointMimic`, so this is real, not
assumed). Publishing the dependents as well is redundant and fights RSP.

Core reports `JointFeedback::gripper` normalized 0 (open) → 1 (closed), so:

```
robotiq_85_left_knuckle_joint = gripper * 0.8      // [0,1] -> [0.0, 0.8] rad
```

**Velocity and effort are NaN**, which is the `sensor_msgs` convention for "no
measurement", not zero — zero would be indistinguishable from "not moving".

> **Corrected 2026-09-01**, against core's `2026-09-01-gripper-tier-design.md`. This spec
> originally claimed `GripperCyclicMessage`'s motor feedback carries `velocity()` and
> `force()`, and that filling both in was a small future core change. **That was wrong
> about force.** `MotorFeedback` has no force field — it carries `current_motor`. The
> `force()` accessor in that header belongs to `MotorCommand`, where force is a current
> *ceiling*, not a setpoint. `GripperMode` has no force mode at all, so no force servo
> exists on this hardware by any path.
>
> So the two fields part company:
>
> - **velocity stays NaN too.** *(Amended 2026-09-01, after the measurement.)* This bullet
>   originally said velocity becomes real once core surfaces gripper feedback. It does not.
>   `MotorFeedback::velocity` was measured on the arm to be the **commanded speed echoed
>   back** while the gripper considers itself moving, and 0 otherwise — unsigned, and
>   identical opening and closing. While the fingers were being physically stopped by an
>   object, the position increments shrank while that field held exactly the commanded
>   0.5000; across runs `|velocity| / |dpos/dt|` was 0.625 closing on air versus 1.060
>   closing on the object at the same commanded speed. A real measurement gives one
>   constant.
>
>   Core has therefore **removed** `GripperFeedback::velocity` outright rather than
>   documenting it — a field carrying no information the caller already has is worse than
>   no field, because it looks like a measurement and someone eventually publishes it as
>   one. There is nothing for this package to surface.
>
>   If a gripper velocity is genuinely wanted in `/joint_states`, differentiate `position`
>   and say in the README that it is a derived rate, not a reported one.
> - **effort stays NaN permanently.** `sensor_msgs/JointState.effort` is documented as
>   N·m or N, and core's gripper effort is a normalized 0..1 fraction derived from motor
>   current. Putting it there would mislabel it — the same mistake this package refuses
>   for the tool wrench on `EeState`. Normalized effort belongs on a future
>   `/gripper_state`, where its units can be stated, alongside the raw current in amps.

Some consumers handle NaN poorly; the README says so.

Two honest limits, documented rather than hidden:

- **The mimic model is a free-space approximation.** The 2F-85 is underactuated; on
  contact the linkage deviates so the fingertips stay parallel. TF through the fingertips
  is not trustworthy *during* a grasp.
- **KORTEX percent is not a calibrated aperture.** The four-bar makes aperture-vs-angle
  non-linear. Fine for visualisation; nobody should read grasp width in millimetres off it.

## Launch

**`description.launch.py`** — args `gripper`, `camera`, `prefix`. Runs `xacro`, publishes
`/robot_description`, starts `robot_state_publisher`. Nothing arm-specific, no driver, so
a client wanting only TF can include it.

**`bringup.launch.py`** — includes the above, then starts `kinova_gen3_node`. Exposes the
driver's CLI as launch arguments (`sim`, `ip`, `urdf`, `cpu`, `rt_priority`, `rate`,
`max_ref_speed`) and its parameters (`arbitration_mode`, `estop_clear_max_age_s`). `urdf`
defaults to this package's expanded file, so the driver and TF use the *same model* by
default — which is the whole point.

## Verification — the part that matters

Replacing a hand-edited model with a generated one can silently change kinematics or
inertials, and four things depend on them: gravity compensation, cuRobo's planning, the
new in-loop IK, and the DLS twist solve. A visual check in RViz proves nothing about any
of that.

**A numerical parity check between the old and new models, as a test:**

1. Load both URDFs in Pinocchio.
2. Over ~1000 pseudo-random configurations in joint limits, compare:
   - **FK at the EE frame** — `gen3_end_effector_link` in the old model against
     `end_effector_link` in the new one, since the rename is part of this change.
     Position and orientation, tolerance `1e-9`: these should be bit-comparable if the
     kinematics are unchanged, and any difference is the rename having landed on a
     *different* frame rather than the same one renamed.
   - **Gravity torques** `gravity(q)` — tolerance `1e-9`. This is the inertial parameters
     made observable.
   - **The Jacobian** at the EE frame.
3. Compare joint limits and link masses element-wise.

**A non-zero difference is a finding, not a failure to tolerate.** If the generated model
differs, we need to know exactly where and decide deliberately — the upstream description
may well be *more* correct than the c3pzero edit, but that is a decision to make with the
numbers in hand, not a diff to wave through. The check runs once during implementation and
is kept as a test so a future upstream bump cannot move the model silently.

**Separately, verify the `ee_frame` rename** by asserting that FK through the renamed frame
equals FK through the old name in the old model, to the same tolerance.

**And verify TF actually updates:** with the node running in sim, publish a joint
trajectory and assert that `tf2` reports a *changing* transform from `base_link` to the
tool frame. That is the defect this package exists to fix, so it gets a test rather than
an eyeball.

## Out of scope

- **Gripper commanding.** This publishes gripper *state* into `/joint_states`; commanding
  is its own tier and its own spec.

> **Corrected 2026-09-03.** Three bullets here were stale or wrong and have been removed.
> Recorded rather than silently deleted, because two of them would have caused real work:
>
> - *"the `GripperSink` port this repo would talk to has not [landed]"* — it has. Core
>   `main` carries `GripperSink` (`on_gripper_setpoint` / `on_query_gripper`),
>   `GripperSetpoint`, `GripperState` and `GripperController` since #35.
> - *"Filling in gripper velocity ... blocked on core surfacing `GripperFeedback` through
>   `ArmState`"* — gripper state never arrives on `ArmState`. It is a separate **pull**,
>   `GripperSink::on_query_gripper()`, so nothing is blocked. And velocity is not pending
>   at all: core **removed** the field, having measured it to be the commanded speed
>   echoed back rather than a measurement. Gripper velocity is NaN permanently, for the
>   same reason effort is.
> - *"[`robot_state_publisher`] does not derive mimics, so the driver must expand the one
>   actuated angle through the ±1 multipliers itself"* — **false, and it contradicted the
>   "Gripper joint state" section above.** Verified empirically on 2026-09-03 with a
>   two-joint URDF (one revolute driver, one `<mimic multiplier="-1">`): publishing only
>   the driver joint moved the mimic link by exactly −0.6 rad for a +0.6 rad drive. RSP
>   derives mimics. The original decision stands: **publish the single actuated joint.**
>   TF was also emitted while one of the two movable joints was never published, so the
>   companion claim — no TF until every movable joint is present — is false outright, not
>   just narrowed to independent joints. **Corrected again, 2026-09-03:**
>   `robot_state_publisher` publishes **per-segment**, not all-or-nothing for the whole
>   robot: with two independent revolute joints where only one was published,
>   `base->link_a` TF resolved and moved while only `base->link_b` was absent. A missing
>   joint costs only its own child link's TF, never the rest of the tree.
- **Real2sim URDF tuning.** Mass, friction and tool-frame calibration against the real
  arm. This package makes it *possible* by giving the model a reproducible source, and the
  parity harness above is the natural place to hang it, but the calibration itself is
  separate work.
- **Moving core's `models/` out.** Core keeps its copy for ROS-free tests. Consolidating to
  one source of truth needs core to depend on an ament package, which is a bigger change
  than this earns.
- **MoveIt configuration.** A `kinova_gen3_moveit_config` is the obvious next sibling, but
  nothing here needs it.
