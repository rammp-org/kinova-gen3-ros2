# Changelog

## Unreleased

> **BLOCKER: this branch does not build from its own declared sources.**
> It needs three dependency releases that do not exist yet. Until they are cut
> and the pins below move, a clean `vcs import` + `docker build` of this commit
> fails to compile `message_mapping.cpp` and `curobo_plan_client.cpp`.
>
> | Dependency | Pinned today | Required | Branch carrying the work |
> |---|---|---|---|
> | `kinova-gen3-driver` (`kinova_gen3.repos`) | `v1.1.1` (no `TrajectoryGoal::speed_scale`, no `kMinSpeedScale`) | the release containing `speed_scale` | `feature/69-trajectory-speed-scale` |
> | `RAMMP-CuRobo` (`kinova_gen3.repos`) | `v1.0.0` (`PlanToPose.action` lacks `axis_lock`/`approach_via`) | the release containing pose constraints | `feature/16-pose-constraints-and-via-point` |
> | `rammp-interfaces-ros2` (base image `rammp-base:1.0.0-jp6` in `Dockerfile`) | 1.1.0 | 1.2.0 | `feature/38-lock-via-speed` |
>
> How it *was* verified: a four-source colcon workspace (this node plus the
> three branches above) built inside the released node image on the Jetson; the
> tests passed there. The code is tested, not merely untried, but it is only
> reproducible from those branches, not from the pins.

### Added

- `GoToEEPose` accepts `axis_lock` (a `ToolAxisLock`), `approach_offset` (an
  `ApproachOffset`) and `speed_scale`. `GoToJointConfig`, `GoToPreset` and
  `ExecuteJointTrajectory` accept `speed_scale` only. The two new arm messages
  are `ToolAxisLock` and `ApproachOffset`. All fields default to off, so
  existing goals behave as before.
- `speed_scale` is passed to the driver's `TrajectoryGoal`, which runs the
  trajectory slower by dilating its executor clock. An out-of-range value is
  refused with a reason in the node log, not clamped.
- Interface change: fields appended at the end of existing messages with
  defaults, plus two new messages. A **minor** bump of `rammp-interfaces-ros2`
  (1.2.0), under Cyclone DDS.

### Known limitations

- A refused goal returns a bare action rejection with no payload; the reason is
  logged server-side only (`kinova-gen3-ros2#39`).

### Fixed

- `kinova_gen3.repos` stated the interface versioning rule backwards; it now
  points at the authoritative policy in `rammp-interfaces-ros2`.
