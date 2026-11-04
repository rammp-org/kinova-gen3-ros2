# Unexpected-contact detection: design

Status: approved by Swapnil, 2026-10-29. Requirement: kinova-gen3-ros2#24. Umbrella:
rammp-deployments#88. Most of the work lands in `rammp-org/kinova-gen3-driver`
(`dev`); this repo gets the ROS surface.

Calls marked **(my call)** were made by the designer, not agreed with Swapnil.
Everything else was decided with him.

## Why

The December demo feeds a person at their face. The arm must not hurt them, or
anything else, when it touches something nobody planned for: an obstacle the
planner does not know about, or the user themselves outside a bite transfer.
On a large unexpected force the arm stops, yields, and stays out of the way
until someone deliberately clears it.

The old detector (Demo-Software `collision_checker.py`, PR #176) is the cautionary
example. It ran at 100 Hz in a ROS node, used `qdd = 0` so fast motion looked like
contact, put one 30 N·m threshold on every joint (the wrist joints are capped at
9 N·m in the driver, so it could never fire from them), stopped stiff and kept
pressing on what it hit, went silently off when the model failed to load, and was
auto-cleared at 1 Hz by a behaviour node. This design starts from scratch; those
are the lessons.

## Goals

- Detect contact anywhere on the arm from the joint-torque sensors, without
  knowing where the contact is.
- React in the driver's RT loop, independent of any client being alive.
- After a trip, float: the arm yields to any push and stays where it is left.
- Latch until a deliberate clear. A client can tell "stopped on contact" apart
  from "goal failed" and "e-stopped".
- Thresholds are runtime-switchable: named profiles or exact per-joint values,
  always under a backstop ceiling.
- Documented false-positive behaviour under fast motion and payload; bench
  evidence on the arm (the #24 acceptance list).

## Non-goals (this round)

- Contact localization. The sensing cannot do it; nothing here may depend on it.
- Resetting thresholds to default on halt, handover or recovery: rammp-org/kinova-gen3-driver#102.
- Who may clear a contact stop. Swapnil is taking this to the team; v1 uses the
  existing `/clear_faults` path (below) and does not restrict callers beyond what
  that service already does.
- Payload compensation (a declared grasped mass in the model). See Payload.
- Admittance / hand-guiding (item 4 of the momentum-observer handoff).
- A speed cap near the user. It belongs to the limits part of rammp-deployments#88.
- Guarding the face during a bite transfer. The capability engine owns that with
  its own force check on `/ee_state`; the driver keeps only a backstop there.

## Anti-goals

- A latch that anything clears automatically.
- Detection that is off without saying so, loudly and continuously.
- A stop that keeps pushing into what it hit, or springs back toward it.
- A detector so twitchy in normal demo motion that people learn to turn it off.

## What we know from the arm

From alexchunlin's `feat/momentum-observer-demo` (`kinova_gen3_ros2/test/momentum_observer/`,
`HANDOFF.md` 2026-10-29), measured on abra (core v1.2.0, old URDF masses):

- The quasi-static residual `g(q) − τ` spikes to ~4.5 N·m on the shoulder at every
  leg start and stop with nobody touching the arm. The momentum observer removes
  that inertial part; its floor in motion was ~3 N·m, set by the mass model.
- Still-arm sensor noise at 1 kHz is 0.02 N·m RMS. 1 kHz did not lower the motion
  floor versus 100 Hz: the floor was unmodeled dynamics, not sample rate.
- Finger pokes (1–4 N·m on the shoulder) were inside that floor. Sustained pushes
  of ≥5 N·m stood out.

Since then the mass model has been validated (driver
`feat/gravity-model-identification`, PR #88): held-out residual ≤0.5 N·m RMS on
j1–j4 and ≤0.2 N·m on j5–j7. Per Swapnil, the masses did not change, so the
detector does not wait on #88 and works on whatever model and offsets the driver
runs. Re-measuring the motion floor on the arm is still the first on-arm step,
because the thresholds come from it.

**Relation to the teammate's work.** This builds on it rather than around it: the
same De Luca momentum observer, the same sign convention, their recommendation to
put it in core's RT loop, their Python observer and replay harness as the
reference the C++ port is tested against, and their recorded runs as test data.
One departure: no baseline-map subtraction **(my call)**. That map corrected the
old model's pose-dependent error; with the fitted model the threshold should
absorb what is left. Revisit if the re-measured floor says otherwise.

## Design

### 1. Signal: per-joint momentum-observer residual

Generalized-momentum observer (De Luca et al., IROS 2006), exactly as derived in
the teammate's `observer.py`, ported to C++ on the driver's own `Dynamics`
(so it uses whatever URDF and offsets the driver is running):

```
r = K ( p − p0 − ∫ ( τ + Cᵀ(q, q̇) q̇ − g(q) + r ) dt ),   p = M(q) q̇
```

`τ` is the transport's torque after the dev sign flip and whatever torque offsets the driver is configured with
(command-signed: `τ ≈ g(q)` at rest). `r` tracks the external joint torque through
a first-order filter of bandwidth `K`.

Detection is on the seven joint residuals, **not** on `ArmState::ee_wrench`. The
wrench assumes the push lands at the hand; a push on the elbow mapped to an
end-effector wrench can come out small or meaningless. `ee_wrench` stays as it is
for diagnostics and for the capability engine.

- `K`: start at 25 rad/s (40 ms time constant) and tune on recorded data
  **(my call)**. Higher K is faster but passes more of the arm's 5–15 Hz motion
  content into `r`.
- Start-up: `r` is not trusted until `5/K` after (re)initialization. During that
  window the monitor reports `settling` and cannot trip. It re-initializes at each
  `RtExecutor::run` start and on a clear, not on control-mode changes: the observer
  reads only sensors and the model, so the command mode does not affect it, and a
  re-init at every goal's mode change would leave a blind window at each goal start.
- A missed exchange (executor `missed` flag) does not step the observer; the next
  good sample integrates with the true `dt` **(my call)**. **(Updated 2026-10-30
  after review:** the executor passes the measured time between feedback reads,
  so misses, overruns and the `set_actuator_modes` handshake count; a step is
  capped at `1/K` (40 ms). At `K·dt = 1` the Euler step lands on the external
  torque of a still arm, while an uncapped `K·dt > 1` overshoots. The observer is
  not re-seeded on a gap: the handshake likely runs past 40 ms on the arm, so a
  re-seed would leave a `5/K` blind window at every goal's mode switch, which the
  bullet above rules out.**)**

### 2. Where it runs

| Piece                                                                                  | Lives in                                                                                      | Why                                                                                                                                  |
| -------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `ContactMonitor`: observer, thresholds, debounce, latch                                | driver RT loop, every cycle after feedback                                                    | Safety must not depend on a non-RT thread being scheduled (the pump shares `dyn_mtx_` with the sampler). Reaction in the same cycle. |
| Immediate reaction                                                                     | driver RT loop                                                                                | See 3.                                                                                                                               |
| Thresholds, profile, mode in; latch state out                                          | Supervisor, through a lock-free handoff (the `Seqlock` pattern already used by `FeedbackTap`) | Policy and conversation stay out of the RT path.                                                                                     |
| `HaltReason`, refusing commands, clear                                                 | Supervisor                                                                                    | Existing halt/fault vocabulary.                                                                                                      |
| Service, diagnostics, debug topic **(updated 2026-10-30**: was a status topic\*\*)\*\* | this repo's node                                                                              | ROS surface only; no detection logic.                                                                                                |

RT rules from the driver's CLAUDE.md apply: no allocation, lock or block in
`ControlMode::compute` or the executor cycle; extend `rt_safety_test.cpp` to cover
the monitor; measure cycle-time cost on abra before and after, don't assume.
`M(q)` and `Cᵀq̇` every cycle is new RT load; if it is too slow, find a cheaper
formulation, not a slower rate.

Trip rule **(my call)**: trip when any joint has `|r_i| > θ_i` for `N` consecutive
good cycles (start N = 5, i.e. 5 ms). `r` is already filtered; the debounce only
rejects isolated bad samples. The trip record holds: the cycle count, joint, `r`
(all seven), `θ` (all seven), `q`, `q̇`. The profile and the detection mode are on
`/contact_monitor`, not in the record **(updated at implementation)**. **(Updated
2026-10-30, Swapnil's ruling:** `/contact_monitor` is removed; the profile and the
detection mode are in the `Contact detection` diagnostics, with the whole record.**)**

### 3. Reaction

**Float.** A new `ControlMode`, pre-constructed at startup (no allocation on trip):

```
τ_cmd = g(q) − D q̇ − K_s (q − q_a)
```

with a dragged anchor: `q_a` starts at the measured `q` at the trip; whenever
`|q_i − q_a,i| > δ`, `q_a,i` is dragged to stay `δ` behind `q_i`. So the arm yields to
any push, slows under joint damping `D`, and stays where it is left, while the spring
resists creep from model error with at most `K_s·δ` per joint. Gains clamp under
the existing per-joint torque limits. Starting values (tune on the arm):
`δ` = 2°, `K_s·δ` ≤ 1 N·m on j1–j4 and ≤ 0.3 N·m on j5–j7, `D` chosen to stop
the arm from demo speeds within ~0.2 s. **(my call: the dragged anchor is how
"stays where it is left" is met without free drift.)**

- **Move running in torque or impedance mode:** the RT loop switches to float in
  the trip cycle.
- **Move running in position mode:** the RT loop adopts float at the next cycle
  boundary through the same path every mode change already takes
  (`RtExecutor::run` step 3 calls `set_actuator_modes`, a blocking pump the driver
  already accepts at a boundary on a mode change). No Supervisor round-trip. In
  the trip cycle itself the command is the current position (a freeze). While
  latched, the executor ignores `request_mode` so nothing can switch the arm out
  of float; only the clear releases it. (Corrected 2026-10-29 from "the Supervisor
  switches it": the code shows the executor does mode switches itself.) Today the
  switch also drops actuator torque for ~160 ms (#92). #94 fixes that; Swapnil
  considers it close enough to build against. Position mode is the common case
  (the idle hold and the post-clear hold are position), so this path is the main
  one, not an edge case. Measure the window on the arm. If it is long, the
  recommendation is that moves near the user run in impedance mode (the capability
  engine already supports `control_mode: IMPEDANCE`).

**Gripper.** Nothing is commanded on a trip: as on any halt, the driver stops
commanding the gripper and it keeps its current grip; it is never commanded open
**(updated at implementation**: an in-flight gripper goal does not finish\*\*)\*\*. While latched, new gripper goals are refused like arm goals, so nothing
changes until a clear **(my call)**.

**Latch.** The arm stays in float until cleared. While latched:

- In-flight goals and streams settle with a distinct result (below). An open stream
  closes with `StreamCloseCause::kContact`, not `kHalted` **(updated at
  implementation)**.
- New motion goals, stream opens and gripper goals are refused with the same reason.
- A separate `Contact detection` diagnostics task, next to `Arm link`, shows ERROR
  `stopped on contact: joint N, |r| = x N·m > θ = y N·m at the trip (profile now P); call /clear_faults` with the trip record **(updated at implementation;
  "at the trip" and "now" added 2026-10-30 after review, since the profile can
  change while latched)**.

A contact stop is **not** an `ArmLink` `kFaulted` state: faulted hands the arm back
to its base, and here the driver keeps control to float. It is a halt with a new
reason plus a latched sub-state.

**Clear.** Through the existing `/clear_faults` service, extended to clear a contact
latch **(my call; who may call it is the team's open question)**. On clear the
Supervisor does what it already does after recovery: hold at the measured pose in
its normal hold. Nothing in the driver or node clears it on its own.

**Latch versus other faults and halts (my call).** The contact latch survives
everything except a clear: ArmLink faults, link loss and reconnect, e-stop, and
ownership changes. If the arm comes back from an ArmLink outage with the contact
latch set, the Supervisor resumes float, not the position hold, and goals stay
refused. One `/clear_faults` call clears whatever is latched (ArmLink fault,
contact, or both). It is extended to accept a contact-only latch while the link
is `kRunning`, where today it refuses anything but `kFaulted`. If someone clears
while still pressing on the arm, it trips again, which is safe.

Detection is armed whenever the executor runs, including the idle hold between
goals: touching a stationary arm trips it like touching a moving one.

**Result code.** New `HaltReason::kContact` and a new result code
`kContactStop = −11` (−10 is taken), string `halted: unexpected contact` **(my
call)**; the joint is in the trip record, not the string **(updated at
implementation)**. Today every halt is `−9 HALTED` and clients can only tell them apart
by string. A contact stop needs a different recovery (a clear), so clients must be
able to branch on it. Clients that treat `−9` as "something stopped me" (capability
engine `arm.py`, test scripts) must learn `−11`; the comment lists in
`rammp_arm_interfaces` actions get `CONTACT_STOP=-11`.

### 4. Thresholds and profiles

Mirrors the driver's `GainsProfile` pattern:

```
enum class ContactProfile { kDefault, kSensitive, kLoose, kCustom };
```

**(Updated 2026-10-30 after review:** `kDefault` is 0, as is `PROFILE_DEFAULT` on
the wire, so a request that leaves `profile` unset selects the default, not the
most sensitive profile.**)**

- Named profiles are per-joint threshold vectors from startup config. Changing
  one means editing config and restarting **(my call)**: a ROS parameter set
  cannot carry the control token, so live parameter edits would let any process
  retune the arm.
- `kCustom` carries exact per-joint values. Live tuning on the arm goes through
  `kCustom` on the token-gated service.
- A per-joint **backstop ceiling** from startup config bounds every profile and every
  custom value. It is not runtime-editable **(my call)**.
- **Refuse loudly, always.** A custom value above the ceiling, a non-finite or
  non-positive value, a named profile edited above the ceiling, or a caller
  without the control token: the request is refused with an error naming the joint
  and the limit. Nothing is clamped silently. Invalid startup config is fatal.
- Switching is one call and holds until the next one. No expiry (Swapnil's call).
  It takes effect within one cycle via the handoff. Gated on the control token,
  like every other command **(my call)**.
- The driver starts in `kDefault`. Resetting on later events is #102.
- `bite_transfer` is not a separate mechanism: the capability engine switches to
  `kLoose` (or custom values) for the transfer, which leaves only the ceiling. It
  switches back afterwards.

**Starting numbers.** Thresholds are tuned together with Swapnil on the arm. The
starting point is a trip at ≤30 N at the tool or forearm along the feeding
approach, comfortably under ISO/TS 15066's face limit.

**(Updated 2026-10-30 after the push tests, Swapnil's ruling:** sensitive
`[8,8,6,6,2,2,2]`, default `[16,16,12,12,4,4,4]`, loose `[24,24,18,18,6,6,6]`,
ceiling unchanged `[30,30,22,22,7,7,7]` N·m. The position-mode tour at speed 1.0
peaked at 5.9 N·m on j1 in the 50 Hz `/contact_monitor` samples; the 1 kHz loop
went past 8 (a would-be trip at j1 8.01 against the first default, now `sensitive`). At home the new default trips at about 23–48 N at the gripper,
depending on direction. Measured floors: `docs/interface.md`, False positives.**)**

Turning a force target into joint thresholds:

1. `θ_i(default) = max(m × motion-floor peak_i, small absolute minimum)`, with the
   floor measured on the arm with nobody touching it at demo speeds (m ≈ 3 to start).
1. Pose check, offline from logged `q` along the feeding approach: for contact
   points on the tool and forearm, compute the smallest force (worst direction)
   that crosses some `θ_i`. That is the detector's real force threshold at that pose.
   If it exceeds the target anywhere, lower `θ` or change the approach.
1. Known blind spot: a force along a line through the joint axes (near-extended
   arm, push along the arm) produces almost no joint torque. The pose check
   reports where the approach goes near such poses; the fix there is the approach.

### 5. Payload

A grasped object's weight shows up in `r` as external torque. A spoon (~50 g)
is below the floor. A filled container (~0.5 kg) puts a few N·m on the shoulder
and may trip `kDefault`. v1 handles it with profiles: the client switches to a
looser profile while carrying **(my call)**. The false-positive section of the
docs reports measured `r` with the demo's real payloads. Declaring payload mass
to the model is follow-up work.

### 6. Modes and the deliberate disable

Startup parameter `contact_detection: off | observe | enforce`, default `enforce`.

- `observe` runs everything and reports would-be trips (diagnostics
  WARN, log) without acting. This is how the false-positive runs are done safely.
- `off` and `observe` show WARN in the `Contact detection` diagnostics continuously
  **(updated at implementation**: its own task, not `Arm link`**)**, and print at
  startup. Not switchable at runtime.
- If `Dynamics` or the monitor cannot be built in `enforce`, the driver fails
  to start. Nothing degrades silently.

### 7. ROS surface (this repo)

- Service `/set_contact_thresholds`: profile enum, optional seven custom values,
  token. Response: accepted, error string, the thresholds now active.
- Topic `/contact_monitor` at the state rate (~50 Hz): `r` (7), active `θ` (7),
  profile, mode, `settling | armed | latched`, the last trip record. **(Updated
  2026-10-30, Swapnil's ruling:** removed before release, with its message
  `ContactMonitorState`. It duplicated the diagnostics and overlapped `/ee_state`,
  and a published type is hard to remove once clients use it. The diagnostics carry
  the state and the whole trip record (`last_trip_thresholds_nm`, `last_trip_q_rad`,
  `last_trip_qd_rad_s` added). For tuning, the read-only parameter
  `contact_debug_topic` (default `false`) publishes `sensor_msgs/JointState` on
  `/contact_monitor/residual` at 50 Hz, `effort` = `r`; it is not created when false
  or with detection `off`.**)**
- `Contact detection` diagnostics as above **(updated at implementation**: its own
  task, not `Arm link`**)**. `docs/interface.md` gets the service, debug topic,
  result code, parameters and a "contact stops" section.
- The service goes in `rammp-interfaces-ros2` **(updated 2026-10-30**: no message
  any more\*\*)\*\*.

## Verification

Off-arm:

- C++ observer vs the teammate's Python `MomentumObserver` on their recorded runs
  (100 Hz runs and the 1 kHz `static1k`/`base1k`, on abra under `~/momentum_observer/runs/`):
  agreement within a stated tolerance after the sign convention is matched.
- Synthetic sweep with injected pushes (their `synthetic.py`): trip on the push,
  no trip without.
- `SimTransport` tests: trip → float → latch → refusals → `−11` → clear → hold, for
  torque and position mode; refusal cases for every bad threshold request;
  `observe` never acts; `enforce` without a model fails startup.
- `rt_safety_test.cpp` covers the monitor and float mode (no allocation).

On the arm. Every motion batch is described to Swapnil (what, speed, position or
impedance) and confirmed before it runs; the arm and abra are shared.

1. No motion, `observe`: still floor per joint on the fitted model.
1. Demo-speed and fast tours, nobody touching, `observe`: motion floor per joint →
   default thresholds. Count would-be trips: the false-positive number for the docs.
1. Still arm, slow hand pushes, `observe`: the quasi-static wrench is exact at
   rest and serves as the force reference for the threshold-to-force mapping.
1. Slow moves, `enforce`, hand pushes (Swapnil): trips, floats, latches, clears.
   Then demo speed.
1. Measure force build-up after a trip and the position-mode stiff window.
1. Real payloads (spoon, container), `observe`: their effect on `r`.
1. Pose check over the logged feeding approach.

Results go in a bench report linked from #24.

## Dependencies

- #94 (bumpless mode switch), treated as close enough to build against. Build on
  `dev`; rebase onto #94 when it lands.
- Not #88: the masses did not change (Swapnil).

## Open, outside this design

- Who may clear a contact stop (Swapnil, with the team).
- Client adoption: the capability engine must handle `−11` and switch profiles for
  the bite transfer, payloads, and any other deliberate contact (for example
  scooping or pressing a utensil onto a plate). Who does that, and the full list of
  deliberate contacts in the demo, are not settled. Until they are, `kDefault` may
  trip during those motions; the `observe` runs will show where.

## Left to the builder

- Splitting the work into PRs across the driver, interfaces and this repo.
- The observer's exact C++ form (e.g. how `Cᵀq̇` is computed) as long as it is
  RT-safe and matches the reference.
- Message field names and layouts; the trip-record format.
- Float damping and spring values within the stated intent, from the arm.
- How the RT loop selects float (pre-built mode, how the executor swaps to it).
