# Streaming tier over ROS 2 (design)

**Date:** 2026-08-29
**Status:** Design (approved by user; ready to plan/implement)
**Repo:** `rammp-org/kinova_gen3_ros2`, on `feat/ros2-command-tiers`
**Core dependency:** `kinova-gen3-driver` PR #29 (`feat/streaming-tier`)
**Predecessor:** `2026-08-29-ros2-arbitration-tier-design.md` (built, PR #13). Opening a
stream requires ownership, so that tier is load-bearing here, not merely adjacent.

## What this is

**Spec 2 of 2.** Exposes core's `StreamSink` — session open/close plus setpoint delivery —
as ROS services and topics, so teleop and reactive control can drive the arm at rate.

The command tier is built for trajectories: plan a path, hand it over, let it run.
Streaming is the opposite shape — a setpoint every cycle from an external source, with no
plan and no "done".

## The decision that shapes everything: controllers, not pairs

Core models a session as a `(SetpointKind, ControlModeKind)` **pair**. Exposing that
directly over ROS was tried and rejected during design, for two reasons:

1. **It makes illegal states representable.** Two independent enums give 20 combinations,
   5 legal. That is the same defect core designed *out* of its setpoint structs — "there
   is deliberately no tag field on the setpoint itself, so 'kind says pose, pose field is
   garbage' is not representable." Two fields at the session level puts it back.
1. **It does not survive the next controller.** Impedance-plus-feedforward-torque is not a
   cell in the matrix — it is one controller taking *two simultaneous inputs*. Any flat
   enum answer to that is a name like `JOINT_POSITION_COMPLIANT_PLUS_FF`, and the enum
   stops describing anything.

**Decision.** The ROS surface names a **controller** (a control law) and a set of
**channels** (input topics it reads). A session opens on a controller; the driver replies
with the channels to publish on.

- A client never states a `(kind, mode)` pair, and never restates something the driver
  already knows. It names a controller; the driver names the topics.

- Adding a controller is a table entry plus, at most, a message type — never a schema
  change to `OpenStream`, which is why `controller` is a **string** rather than an enum.

- `channels` is a **list from day one**. Core admits exactly one `SetpointKind` per
  session, so a controller whose registry row declares more than one channel cannot be
  opened today and is reported `available: false`. The rule applies to the *controller's*
  channel list, not to client input — the client never sends channels. When core learns to
  admit a set, that single validation relaxes and nothing else changes.

  This is the second reason `cartesian_impedance` is unavailable: even once core has
  `kEeWrench`, a controller reading both `pose` and `wrench` needs multi-channel sessions.

Note what does **not** grow: message types. Joint position, velocity and torque all have
the same shape (`float64[7]`). Four payload shapes cover every channel.

## Architecture

`StreamServer` mirrors `ArbitrationServer`: it holds a `kinova::interface::StreamSink&`
and nothing else from core, so it unit-tests against a fake with no robot, no URDF and no
threads. It is wired to the **Arbiter**, not the Supervisor — that is how setpoint tokens
get checked at all.

```
  ArbitrationServer ── ArbitrationSink* ──┐
                                          ├──> Arbiter ──> Supervisor ──> modes
  StreamServer ─────── StreamSink* ───────┘
                                          ▲
  Ros2Backend / GoTo servers ─ CommandSink┘
```

### Controller registry

A static table in `StreamServer`, one row per controller, mapping to core's pair. It is
the only place the collapse lives.

| controller            | channel          | core pair                       | available |
| --------------------- | ---------------- | ------------------------------- | --------- |
| `joint_position`      | `joint_position` | `kJointPosition` × `kPosition`  | yes       |
| `joint_impedance`     | `joint_position` | `kJointPosition` × `kImpedance` | yes       |
| `ee_pose_impedance`   | `pose`           | `kEePose` × `kImpedance`        | yes       |
| `joint_torque`        | `joint_torque`   | `kJointTorque` × `kTorque`      | yes       |
| `joint_velocity`      | `joint_velocity` | `kJointVelocity` × `kVelocity`  | no        |
| `ee_twist`            | `twist`          | `kEeTwist` × `kVelocity`        | no        |
| `cartesian_impedance` | `pose`, `wrench` | — (no `kEeWrench` in core)      | no        |

**`available` is computed live from `pair_supported()`, never hand-maintained**, so it
cannot drift from core. `cartesian_impedance` is the exception: core has no `kEeWrench`
kind at all, so there is nothing to ask `pair_supported()` about and the row is marked
unavailable in the table itself, with the reason in its rejection message.

When core lands `JointVelocityMode`, those rows light up with no ROS change. That is the
point of computing availability rather than declaring it.

## Interfaces

```
msg/JointSetpoint.msg    float64[7] values          # rad | rad/s | N·m, per topic
                         uint8[16]  token
msg/PoseSetpoint.msg     geometry_msgs/Pose   pose
                         uint8[16]  token
msg/TwistSetpoint.msg    geometry_msgs/Twist  twist  # [linear; angular], base frame
                         uint8[16]  token
msg/WrenchSetpoint.msg   geometry_msgs/Wrench wrench # [force; torque], base frame
                         uint8[16]  token

msg/StreamStatus.msg     std_msgs/Header header
                         bool     open
                         string   controller         # "" when closed
                         string[] channels
                         float64  timeout_s
                         uint64   rejected_count

msg/ControllerCapability.msg   string   name
                               string[] channels
                               bool     available

srv/OpenStream.srv       string    controller
                         float64   timeout_s
                         uint8[16] token
                         ---
                         bool      accepted
                         string[]  channels          # the driver names the topics
                         int32     error_code
                         string    message

srv/CloseStream.srv      uint8[16] token
                         ---
                         bool   closed
                         string message

srv/ListControllers.srv  ---
                         ControllerCapability[] controllers
```

**Tokens are per message, not per session.** The point of a token is to validate the
sender; attaching it server-side would make authority per-session and admit a leftover
publisher from a previous session under the new session's authority. This also keeps the
rule from spec 1 intact: *a token rides on every message that can command motion.*

### Topics and QoS

| Topic                      | Dir | Type             | QoS                                           |
| -------------------------- | --- | ---------------- | --------------------------------------------- |
| `/setpoint/joint_position` | sub | `JointSetpoint`  | **best-effort, KeepLast(1)**                  |
| `/setpoint/joint_velocity` | sub | `JointSetpoint`  | best-effort, KeepLast(1)                      |
| `/setpoint/joint_torque`   | sub | `JointSetpoint`  | best-effort, KeepLast(1)                      |
| `/setpoint/pose`           | sub | `PoseSetpoint`   | best-effort, KeepLast(1)                      |
| `/setpoint/twist`          | sub | `TwistSetpoint`  | best-effort, KeepLast(1)                      |
| `/setpoint/wrench`         | sub | `WrenchSetpoint` | best-effort, KeepLast(1)                      |
| `/stream_status`           | pub | `StreamStatus`   | reliable, transient_local, depth 1, on change |

**Setpoint QoS is dictated by core's semantics, not chosen.** From the streaming guide:
*"Dropping an intermediate setpoint is correct, not lossy... 42 already says where the arm
should be, independent of 41 ever having existed."* Reliable-with-queue would deliver
stale setpoints late, which is the exact failure the session deadline exists to catch.

`/setpoint/twist` and `/setpoint/wrench` are subscribed but no controller can currently
accept them. They exist so the surface is complete and so clients can be written against
it; setpoints there are dropped by `admit()` like any other unmatched kind.

## Threading

Four callback groups. The separation is the design, not tuning.

| Group     | Contents                                          | Why                                                                                                                            |
| --------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| e-stop    | `/estop`                                          | Must never wait. Unchanged from spec 1.                                                                                        |
| session   | `open_stream`, `close_stream`, `list_controllers` | **`on_stream_open` sleeps `mode_settle_s` (250 ms)** holding the arbiter mutex.                                                |
| setpoints | the six topics                                    | Must not queue behind that 250 ms open. `MutuallyExclusive` — one logical writer, matching core's single-writer double buffer. |
| default   | actions, status, diagnostics                      | Everything else.                                                                                                               |

Putting session control in the e-stop group would rebuild in ROS the exact stall spec 1
exists to prevent.

## Data flow

```
/acquire_control                         → token
/list_controllers                        → controller + its channel
  create the publisher, let discovery settle          ← BEFORE opening
/open_stream {controller, timeout_s, token}           → channels[]
  publish on the channel faster than timeout_s
/close_stream {token}
```

**Discovery before open is a real requirement, not advice.** DDS discovery can take
hundreds of milliseconds; a session deadline is typically 100 ms. A client that opens
first and then creates its publisher loses its early setpoints and the session expires
before it ever drives the arm. This is why `list_controllers` reports channels at all —
without it, a client cannot know the topic before opening.

## Error surfacing

`OpenStream` is a service, so rejection is visible and typed. `accepted=false` with core's
`kStreamRejected (-10)` and its message verbatim, for: unsupported pair, `timeout_s <= 0`,
a trajectory goal in flight, or a session already open. Two rejections originate here
rather than in core: an unknown controller name, and a controller whose row is unavailable
because core has no kind for it (`cartesian_impedance`).

**Setpoint rejection is silent by construction** — `on_setpoint_*` returns `void`. A bad
token or wrong channel bumps a counter and writes nothing. `/stream_status`'s
`rejected_count` is the only signal a client has, which is why that topic is not optional.

And a rejected setpoint **does not refresh the deadline**: *"a client sending the wrong
shape is not evidence the stream is healthy."* Publishing hard on the wrong channel does
not keep a session alive — it expires and safe-stops. Silent, but safe.

## Testing

**Unit — `StreamServer` against a fake `StreamSink`** (no robot, no Supervisor):

- each of the six topics routes to its matching `on_setpoint_*` method;
- the token arrives at the sink byte-identical to what was published;
- `open_stream` maps controller → the right `(kind, control_mode)` pair;
- an unknown controller is rejected without reaching core;
- `list_controllers` reports channels for every controller and `available` per
  `pair_supported()`;
- `/stream_status` publishes on change and is latched for a late subscriber.

**Integration — real `Arbiter`:**

- a setpoint carrying the granted token delegates; one carrying a stale token is dropped
  and bumps `rejected_count`;
- **`/estop` is honoured while `on_stream_open` is mid-settle** — the scenario core's
  lock-free `estop()` was written for, and which spec 1 could only simulate.

**Regression:** all 65 existing tests stay green; the sim e2e is unchanged.

## Out of scope

- **Multi-channel sessions.** Core admits one `SetpointKind` per session. The schema is
  ready (`channels[]`, a one-line validation); the capability is core's to add, together
  with `tau_ff` on the impedance modes, which neither impedance mode has today.
- **`JointVelocityMode`.** Gates `joint_velocity` and `ee_twist`, and is itself gated on
  running `velocity_probe` on the real arm — no recorded result yet. An `IGNORES` outcome
  would push core toward `tau = g(q) + Kd·(qd_des − qd)`, which changes the *mode*, not
  these topics.
- **`CartesianImpedanceMode` and `kEeWrench`.** Core calls wiring the former "wiring, not
  design"; the latter is named as "a deliberate future sibling."
- **The gripper.** Plumbed at the transport layer (`JointCommand::gripper`,
  `GripperInjector` in `teleop_socket_server`), invisible to the interface tier. Its own
  tier, deferred by decision.
- **`set_gains` / `query_state` services.** Still unexposed, as in spec 1.
