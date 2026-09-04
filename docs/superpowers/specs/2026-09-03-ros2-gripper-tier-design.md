# ROS2 Gripper Tier — Design

Commanding and observing the Robotiq 2F-85 through the ROS2 surface. Core's gripper
tier (`kinova-gen3-driver` #35) is merged and this repo already links it; what is
missing is the ROS surface and the one wiring line that makes it live.

Core contract: `kinova-gen3-driver`'s `2026-09-01-gripper-tier-design.md`.
Sibling specs: the arbitration and streaming tiers, whose conventions this follows.

## What core gives us, and what it refuses to

`GripperSink` has exactly two methods:

```cpp
void         on_gripper_setpoint(const GripperSetpoint&);   // token-gated
GripperState on_query_gripper();                            // never gated
```

Four facts from core drive every decision below.

1. **The gripper is orthogonal to arm control.** `GripperController` is a decorator on
   `Transport`, not a `ControlMode` — it stamps a gripper field into the outgoing frame.
   You can grip during a trajectory, an impedance hold or a velocity stream, and nothing
   about it touches mode switching. So this tier needs no mode negotiation.
1. **It needs no session.** `Arbiter::on_gripper_setpoint` gates on `admit(s.token)` and
   nothing else — there is no `open_stream` equivalent to build. Holding the arm's token
   is the entire prerequisite. The gripper rides the ARM's token by core's spec decision:
   one physical machine, one holder.
1. **There is no force servo, and no "let go".** `GripperCommand::force` is a *current
   ceiling*: the gripper closes at `speed` toward `position` and stalls when it reaches
   the limit. `GripperMode` has no force mode at all. And `GripperController::release()`
   does not open or slacken the gripper — the last submessage stays latched in KORTEX's
   persistent cyclic command and is retransmitted; the 2F-85 is effectively self-locking.
   To open, command `position = 0`.
1. **`speed` and `force` are not sticky.** `set_target` takes all three fields every call
   and clamps each to `[0,1]`. A message carrying only `position` silently commands
   whatever `speed`/`force` its defaults hold.

## Decisions

1. **A setpoint topic, not an action.** `/setpoint/gripper`, joining its six siblings.
   Maps 1:1 onto `on_gripper_setpoint` with no invented semantics.
1. **No `active` field.** See "The `active` trap" below.
1. **The knuckle joint goes in the arm's existing `/joint_states` message**, one stamp.
1. **Normalized data gets its own topic**, `/gripper_state`, where its units can be stated.
1. **Ship both URDFs for now.** Reducing to one needs a core change; deferred to #14.
1. **`articulated` becomes the launch default**, now that something publishes the joint.

## Command path

```
/setpoint/gripper  kinova_gen3_interfaces/GripperSetpoint   (best-effort, depth 1)
  float32   position    # 0 open .. 1 closed
  float32   speed       # fraction of max closing speed
  float32   force       # CURRENT CEILING, not a force setpoint
  uint8[16] token       # from /acquire_control
```

Best-effort depth 1 matches the six arm setpoint topics and core's semantics: setpoints
are absolute and latest-wins, so dropping an intermediate one is correct.

Subscribed on the **Arbiter**, not the Supervisor — that is what makes the token
load-bearing, exactly as `StreamServer` is wired.

**All three of `position`/`speed`/`force` are sent on every message.** The message
documents this: core does not remember the previous `speed`/`force`, so a client that
fills in only `position` is commanding the other two to their defaults, not leaving them
alone. The ROS layer does **not** add stickiness — inventing state core refuses to keep
would make the ROS surface and the driver disagree about what was commanded.

Clamping is core's job (`set_target` clamps each field to `[0,1]`), so this tier does not
re-clamp. It is the one place the documented range is enforced for every caller.

### The `active` trap

`GripperCommand` carries `bool active`, and it is **not** exposed on the ROS message.

The field is real, but it belongs to the *outgoing frame*, not to the caller.
`JointCommand::gripper.active` is the wire-level gate — `kortex_transport.cpp` reads
`if (cmd.gripper.active)` to decide whether to write a gripper submessage — and it
default-constructs to `false` so that every control mode writing a `JointCommand` cannot
accidentally actuate the gripper. `GripperController::stamp()` sets it from its own
`stamping_` flag.

On the input side it does nothing: `set_target()` arms `stamping_` unconditionally and
discards `c.active`. So a client sending `active: false` meaning "stop commanding the
gripper" gets the gripper commanded anyway. Exposing a field that reads as its own
opposite is worse than omitting it, and there is nothing a client could do with it.

## State path

`Ros2Backend::publish_state()` already runs on the pump thread with a fresh `ArmState`.
It gains one `on_query_gripper()` pull.

```
/joint_states   name += robotiq_85_left_knuckle_joint
                position += gripper.position * 0.8      # [0,1] -> [0.0, 0.8] rad
                velocity += NaN
                effort   += NaN
```

`0.8` is the knuckle's URDF upper limit (`<limit lower="0.0" upper="0.8">` in
`robotiq_description`'s `robotiq_2f_85_macro.urdf.xacro`).

Note the node publishes a joint that is **not in its own model** — it loads the frozen
7-DOF URDF, where the Robotiq joints are `type="fixed"`. That is fine and intended:
`/joint_states` is a message, not a model, and the consumer that needs the joint
(`robot_state_publisher`, on the articulated model) is a different process. The node needs
the name and the conversion factor, not the joint.

**Only the actuated joint is published.** The 2F-85 is underactuated: one revolute DOF
and five `<mimic>` joints at ±1. `robot_state_publisher` derives mimics itself — verified
2026-09-03 with a two-joint URDF, where publishing only the driver joint moved the mimic
link by exactly −0.6 rad for a +0.6 rad drive. Publishing the dependents as well would
duplicate knowledge the model already holds and could drift from it.

**Velocity and effort are NaN, permanently.** `sensor_msgs/JointState.effort` is
documented as N·m or N; core's effort is a 0..1 fraction of `kGripperMaxCurrentA`.
Velocity does not exist at all — core removed the field, having measured `MotorFeedback`'s
to be the commanded speed echoed back rather than a measurement. NaN is the `sensor_msgs`
convention for "no measurement"; zero would be indistinguishable from "not moving".

### Stamping, and a deliberate skew

`GripperState::stamp_s` is **discarded**. Core flags the asymmetry: `ArmState::stamp_s` is
*sample* time, set inside the pump when the feedback was captured, while
`GripperState::stamp_s` is *query* time, computed when you ask. Publishing the arm's
sample time is the honest stamp for `/joint_states`' knuckle row, which piggybacks on the
`ArmState` that `Ros2Backend::publish_state()` was already given.

The cost, stated rather than hidden: `on_query_gripper()` performs its own `snap_` load,
so the gripper value may come from a feedback frame up to one cycle newer than the
`ArmState` beside it — a skew of at most 1 ms at the 1 kHz feedback rate. This is
irrelevant for TF and visualization, which is all the joint drives. It does mean the
conformance suite's "same pump tick" invariant covers `/joint_states` and `/ee_state`, not
the gripper column.

**Correction, 2026-09-03: `/gripper_state` itself is not stamped this way.** The
paragraphs above describe the knuckle row folded into `/joint_states`, which does run on
the pump thread with `ArmState`'s sample time. `/gripper_state` is a separate topic
published by `GripperServer`'s own 20 Hz wall timer (`bringup_node.cpp`, decoupled from
the pump thread), stamped with `node->now()` at the moment it fires — not with
`ArmState`'s sample time and not with `GripperState::stamp_s`. That is still an honest
stamp (it is query time, and `GripperState::stamp_s` is also query time — there is no
sample time to inherit here since nothing pulls an `ArmState` for this publish), just not
the mechanism this section originally planned.

### `/gripper_state`

```
/gripper_state   kinova_gen3_interfaces/GripperState   (best-effort, sensor QoS)
  std_msgs/Header header
  float32 position    # 0 open .. 1 closed, normalized
  float32 effort      # 0..1 fraction of kGripperMaxCurrentA (1.0 A), NOT Newtons
  float32 current     # amps, raw
  bool    present
```

The home for what cannot honestly go in `JointState`. `effort` carries a warning in its
comment: a *sustained* grasp reports a **small** effort (~0.05), because the current
spikes on contact and then settles to a holding current. Anything keying off "high effort
means holding something" is wrong.

`present` is load-bearing: without it a missing gripper and a fully-open one are both
position 0. Note core conflates two meanings — "no `GripperController` wired" and "no
gripper attached" — deliberately, since a caller has no use for telling them apart.

### When the gripper is absent

The joint is published **regardless** of `present`. Omitting it would drop the gripper
out of TF entirely — its links would just vanish from the model rather than render at a
frozen or default pose (verified 2026-09-03: with an independent revolute joint never
published, TF for the OTHER segments still resolves; only the unpublished joint's own
child link is missing — RSP publishes per-segment, not all-or-nothing for the whole
robot). `present` on `/gripper_state` carries the truth about presence instead.

A third REP 107 diagnostics task, `kinova_gen3_node: Gripper`, reports the presence and the
current, and WARNs when a gripper was expected but `present == false`.

"Expected" cannot be inferred from the node's own model: the node loads
`kinova_gen3_7dof.urdf`, in which the gripper joints are frozen, so its model never has a
gripper regardless of the hardware. It therefore takes an `expect_gripper` bool parameter,
default `true`, and WARNs on `expect_gripper && !present`. A robot genuinely built without
one sets it false and the task reports OK. Without the parameter the task would either
spam every gripper-less robot or never fire at all.

## Wiring

One line makes the tier live. `deps.grip` is currently `nullptr`, so
`Supervisor::on_gripper_setpoint` returns at `if (!grip_) return;` and **every gripper
setpoint is silently dropped** — the Arbiter admits it, so `rejected_count` does not even
move.

```cpp
GripperController grip(*base);      // decorates Transport, stamps the outgoing frame
FeedbackTap       tap(grip, snap);  // was: tap(*base, snap)
...
deps.grip = &grip;                  // was: nullptr
```

Decorator order follows core's own idiom in `apps/teleop_socket_server.cpp`
(`GripperController injector(*base_transport); FeedbackTap transport(injector, snapshot);`).

## Model and launch

`articulated` flips to `true` by default: the articulated model has 13 movable joints but
only **8 independent** ones — seven arm plus the knuckle — and RSP derives the other five.
Safe on a gripper-less build: the joint is not in the model and RSP ignores joint states
for joints it does not know.

Three docstrings justify the old `false` default with the disproved claim that RSP does not
derive mimics (`description.launch.py` twice, `bringup.launch.py` once). They are corrected,
not just contradicted.

The driver keeps `kinova_gen3_7dof.urdf` — core's `Dynamics` asserts `nv == 7`. One file for
both consumers is #14.

## Out of scope

- **`control_msgs/action/GripperCommand`.** The ecosystem-standard interface, and the thing
  MoveIt drives. Deferred because it must *synthesise* `stalled` and `reached_goal`, which
  core has no notion of, and because the standard is internally inconsistent: its comments
  say position is gap in metres and effort is Newtons, while `GripperActionController`
  passes position straight to the joint. Worth doing, worth its own spec.
- **Any force servo.** The hardware has none by any path.
- **Gripper velocity.** Core removed the field; there is nothing to publish.

## Testing

- **Unit:** normalized→radian mapping including both endpoints; NaN in velocity/effort;
  `active` absent from the generated message.
- **Integration (gtest, fake sink):** a setpoint with the wrong token is refused and bumps
  `rejected_count`; with the right token it reaches the sink with all three fields; the
  joint is still published when `present == false`.
- **Conformance:** a `gripper` section — `/gripper_state` present and self-consistent, the
  knuckle joint appearing in `/joint_states` within the URDF limits, an untokened setpoint
  refused under `enforced`. It commands the gripper, so it belongs in the motion section's
  ordering: nothing runs until the e-stop path is proven.
- **On-arm:** open/close at low `force`, confirming the effort spike-then-settle signature
  and that TF fingers track. Attended, per the runbook.
