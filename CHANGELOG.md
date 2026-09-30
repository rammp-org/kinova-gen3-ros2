# Changelog

## Unreleased

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
