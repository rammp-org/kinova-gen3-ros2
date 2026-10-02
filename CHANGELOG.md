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
> | `RAMMP-CuRobo` (`kinova_gen3.repos`) | `v1.0.0` (`PlanToPose.action` lacks `hold`/`approach_via`) | the release containing pose constraints | `feature/16-pose-constraints-and-via-point` |
> | `rammp-interfaces-ros2` (base image `rammp-base:1.0.0-jp6` in `Dockerfile`) | 1.1.0 | 1.2.0 | `feature/38-lock-via-speed` |
>
> How it *was* verified: a four-source colcon workspace (this node plus the
> three branches above) built inside the released node image on the Jetson; the
> tests passed there. The code is tested, not merely untried, but it is only
> reproducible from those branches, not from the pins.

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
  (1.2.0), under Cyclone DDS.

### Known limitations

- A refused goal returns a bare action rejection with no payload; the reason is
  logged server-side only (`kinova-gen3-ros2#39`).

### Fixed

- `kinova_gen3.repos` stated the interface versioning rule backwards; it now
  points at the authoritative policy in `rammp-interfaces-ros2`.
