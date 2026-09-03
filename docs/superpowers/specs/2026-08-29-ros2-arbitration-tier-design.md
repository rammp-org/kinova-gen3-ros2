# Arbitration / control-ownership tier over ROS 2 (design)

**Date:** 2026-08-29
**Status:** Design (approved by user; ready to plan/implement in a fresh session)
**Repo:** `rammp-org/kinova_gen3_ros2`
**Core dependency:** `rammp-org/kinova-gen3-driver` **PR #29 (`feat/streaming-tier`)**.
Arbitration itself landed on core `main` (PR #25), but `Arbiter`'s constructor gained a
second downstream (`StreamSink&`) in #29, and `Supervisor`'s gained a `JointTorqueMode&`.
This repo already requires #29 for the latter — see the WARNING in `kinova_gen3.repos`.

## What this is

**Spec 1 of 2.** Core grew two new tiers; neither is reachable from ROS. This spec
exposes the **arbitration / control-ownership tier**: who is allowed to command the arm,
and how the arm is stopped. The **streaming tier** (setpoint topics) is a separate spec
that builds on this one, because every streaming setpoint carries a token minted here.

Today `bringup_node` wires `Ros2Backend → Supervisor` directly. The `Arbiter` — which
core wrote, reviewed, and made lock-free on the e-stop path — is not in the chain at all.
Consequences: any node on the domain can command the arm, and there is no way to stop it
from ROS short of killing the process.

This spec adds ownership (acquire / release / revoke), a broadcast e-stop, and the status
and diagnostics surfaces that make both observable.

## Decisions that shape the design (user-approved)

1. **Two specs, arbitration first.** Streaming's token field then already exists, and the
   e-stop path is proven before anything streams setpoints at the arm.
2. **`/estop` is a broadcast topic, not a service.** `true` engages, `false` clears.
   Anyone may publish either. It carries a **custom stamped message**, not
   `std_msgs/Bool` — see "Why not std_msgs/Bool".
3. **`arbitration_mode` is a launch-time, read-only parameter, defaulting to `disabled`.**
   Core has no setter for `ArbitrationMode`; a dynamic parameter would silently no-op.
4. **`set_gains` and `query_state` are out of scope.** Separate follow-on.
5. **`kRejectUnauthorized` maps to a plain rclcpp REJECT.** `/control_status` is the
   explanation channel; no accept-then-abort.
6. **Status topic is `/control_status` + `ControlStatus.msg`,** published on change.
   ("Arbitration" is core's internal vocabulary and should not leak into the ROS surface.)
7. **REP 107 `/diagnostics` is in scope,** via `diagnostic_updater`.
8. **Action cancel stays open and unauthenticated.** See "Deliberate deviations".
9. **This is a high-trust system.** Participants are known and cooperating; the tier
   exists to prevent *mistakes*, not malicious actors. `/acquire_control` is called by the
   task orchestrator and nothing else — an operational contract the driver does not
   enforce. Securing against unknown actors is future work, and belongs at the transport
   layer. See "What arbitration is, and is not".

## The rule that governs token placement

> **A token rides on every message that can command motion, and on nothing that stops the
> arm or reads state.**

This is not a stylistic choice — it is exactly what `Arbiter` already does:

| `Arbiter` method | Gated by `admit()`? |
|---|---|
| `on_trajectory_goal`, `on_trajectory_accepted`, `on_trajectory_cancel` | yes |
| `on_set_gains`, all five `on_setpoint_*` | yes |
| `on_query_state` | no — *"never gated — reads are always open"* |
| `estop`, `estop_clear` | no — no `admit()` call at all |

Requiring a capability to **stop** a robot inverts the safety property. `/estop` therefore
carries no token, and neither does `/revoke_control` (the operator override).

## Architecture

### Wiring

`Arbiter` is an in-process C++ object implementing `CommandSink` + `StreamSink` +
`ArbitrationSink`, so it slots into the existing chain. All four action servers re-point
`set_command_sink()` from `&sup` to `&arb`; nothing else about them changes.

```
                    ┌───────── ArbitrationServer (new) ────────┐
                    │ /acquire_control  srv → grant(owner_id)  │
                    │ /release_control  srv → revoke() [token] │
                    │ /revoke_control   srv → revoke() [oper.] │
                    │ /estop            sub → estop()/clear()  │
                    │ /control_status   pub ← status()         │
                    │ /diagnostics      pub ← status() @1Hz    │
                    └───────────────────┬──────────────────────┘
                                        │ ArbitrationSink*
  Ros2Backend       ──┐                 ▼
  GoToEEPoseServer  ──┼─ CommandSink* → Arbiter → Supervisor → modes
  GoToJointConfig   ──┤
  GoToPreset        ──┘
```

`Supervisor` implements **both** `CommandSink` and `StreamSink`, so it is passed twice —
the same idiom core's own tests use (`Arbiter arb{sink, sink, mode, seed}`):

```cpp
interface::Arbiter arb(sup, sup, arbitration_mode);
```

**Rejected alternatives.** Putting arbitration in a *separate ROS node* would force the
gate to run over DDS, adding latency to precisely the e-stop path core spent a commit
making lock-free. Re-implementing the gate in the wrapper would duplicate ordering logic
that is subtle and already reviewed (see the 20-line comment on `Arbiter::estop()`).

### `ArbitrationServer` (new component)

One class, `kinova_gen3_ros2::ArbitrationServer`, owning the ownership/safety ROS surface. It
holds an `kinova::interface::ArbitrationSink*` and nothing else from core — it never
touches `Supervisor` or the modes. Constructed with the node, the arbiter, and the
resolved `ArbitrationMode` (needed only to populate `arbitration_enabled` in status).

Responsibilities, in full:

- serve the three services,
- subscribe `/estop`,
- publish `/control_status` on change,
- register a `diagnostic_updater` task.

It is deliberately thin: every method is a validated translation into one
`ArbitrationSink` call. That keeps it unit-testable against a fake `ArbitrationSink`
with no robot, no URDF, and no `Supervisor`.

### Declaration order in `bringup_node`

Destruction is reverse-declaration, and the existing comment already warns that the
servers must outlive `sup`. Inserting two more objects makes the required order:

```
servers…  →  sup  →  arb  →  arbitration_server
```

so teardown runs `arbitration_server` (stops accepting ROS calls), then `arb` (stops
delegating), then `sup`, then the servers. Any other order lets a late ROS callback
reach a half-destroyed chain.

## Interfaces

All new files land in `kinova_gen3_interfaces`.

### `msg/EStop.msg`

```
std_msgs/Header header    # stamp: lets a subscriber detect a stale latched clear
bool   engaged            # true = engage, false = clear
string source             # who published it (node name / operator id)
string reason             # free text, for the log
```

**Why not `std_msgs/Bool`.** The `std_msgs` package documentation states that its
primitive types "do not convey semantic meaning about their contents: every message
simply has a field called `data`. Therefore, while the messages in this package can be
useful for quick prototyping, they are **NOT intended for 'long-term' usage**." (Strong
discouragement, not a formal deprecation.) Two concrete costs on a safety topic:

- **No timestamp.** Two uses, both real. *Forensics:* when the arm was stopped, and by
  whom, is the first thing anyone asks after an incident, and `/estop` is the only record.
  *Staleness and replay:* a `ros2 bag` replay containing `engaged: false` would otherwise
  clear a live e-stop indistinguishably from an operator doing it — a stamp lets the
  subscriber refuse a clear that is implausibly old.
- **No source or reason.** Core already distinguishes `kEmergencyStop` /
  `kOwnershipRevoked` / `kOperatorRequest`; a bare bool throws away who stopped the arm.

There is no universal ROS 2 e-stop standard to conform to. The nearest community
precedent is `cob_msgs/EmergencyStopState`; `ros2_control` has its own handlers.

### `msg/ControlStatus.msg`

```
std_msgs/Header header
bool   arbitration_enabled   # false = tokens ignored (launch-time param)
bool   estopped              # latched; nothing is admitted
bool   owned                 # does anyone hold the arm?
string owner_id              # "" if none
uint64 generation            # bumps on each grant; detects dispossession
uint64 rejected_count        # commands refused since start
```

Maps 1:1 onto `interface::ArbitrationStatus`.

### Services

```
srv/AcquireControl.srv    string owner_id
                          ---
                          bool     accepted
                          uint8[16] token
                          uint64   generation
                          string   message

srv/ReleaseControl.srv    uint8[16] token
                          ---
                          bool   released
                          string message

srv/RevokeControl.srv     string reason
                          ---
                          bool   revoked
                          string message
```

**Why three services for two core methods.** `Arbiter::revoke()` takes no arguments and
unconditionally dispossesses whoever holds the arm — so "I am done, releasing" and
"operator seizes the arm from a hung client" are the same call in core. Those deserve
different authority:

- `/release_control` takes a token; **the wrapper checks it matches the current owner**
  before delegating, because core cannot.
- `/revoke_control` is the unauthenticated operator override.

**How the wrapper verifies a release token.** `ArbitrationStatus` deliberately does *not*
carry the token — publishing a capability on a status topic would defeat it — so the check
cannot be made against `status()`. Instead, `ArbitrationServer` **retains the token returned by
the most recent successful `grant()`**, which is sound because `ArbitrationServer` owns the only
ROS surface that can change ownership: it is the sole caller of `grant()`, `revoke()`,
`estop()` and `estop_clear()`. It clears the retained token on every ownership-ending
transition (release, revoke, e-stop engage), keeping it in step with the Arbiter's own
state. A `/release_control` whose token does not match the retained one is refused with
`released=false` and does not reach `revoke()`.

Since we deliberately have **no ownership lease**, `/revoke_control` *is* the recovery
path for a crashed client holding the arm. Without it, the only remedy is restarting the
node.

### Existing actions — additive change

`uint8[16] token` is added to the goal of all four existing actions
(`ExecuteJointTrajectory`, `GoToEEPose`, `GoToJointConfig`, `GoToPreset`), alongside the
`sender_id` they already carry:

- `sender_id` stays the human-readable label, passed to `grant(owner_id)`.
- `token` is the capability.

Omitted ⇒ all zeros, which is exactly what `ArbitrationMode::kDisabled` admits.

`uint8[16]` (not a hex string) maps directly onto `std::array<uint8_t,16>` with no
parsing or validation. CLI ergonomics are unaffected in practice: under the default
disabled mode nobody types a token, and an omitted field defaults to zeros.

The `.action` files' result-code comments gain `-8 NOT_AUTHORIZED` and `-9 HALTED`.

### QoS

| Topic | Direction | Reliability | Durability | Depth |
|---|---|---|---|---|
| `/control_status` | we publish | reliable | **transient_local** | 1 |
| `/estop` | we subscribe | reliable | **volatile** | 10 |
| `/diagnostics` | we publish | reliable | volatile | 10 (`diagnostic_updater` default) |

**`/control_status` is latched, and safely so.** We are the publisher. A publisher
offering `transient_local` is compatible with both volatile and transient_local
subscribers, so latching costs nothing and buys the property we need: a client that starts
late or reconnects immediately learns owner, `generation` and estop state without waiting
for the next change.

**`/estop` is deliberately NOT latched, despite being the safety topic.** We are the
*subscriber* here, and ROS 2 durability compatibility is asymmetric: an offered
(publisher) durability must be at least as strong as the requested (subscriber) one.
A `transient_local` **subscription** is therefore **incompatible with a volatile
publisher** — and `ros2 topic pub`, `rqt`, and most ad-hoc scripts publish volatile by
default. Requesting `transient_local` on `/estop` would mean an operator typing

```
ros2 topic pub /estop kinova_gen3_interfaces/msg/EStop '{engaged: true}'
```

silently fails to connect at all. On an emergency-stop path that trap is unacceptable, so
the subscription takes the permissive default and accepts any publisher.

The consequence — the e-stop latch does not survive a node restart — is not a real loss:
the `Arbiter` is constructed unlatched regardless, so there is no state for a durable
message to restore.

`/joint_states` keeps its existing `SensorDataQoS` and is untouched.

There is no REP mandating diagnostics QoS (REP 2003 covers sensor and map topics), so
`/diagnostics` uses the default that `diagnostic_updater` ships.

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `arbitration_mode` | string | `disabled` | `enforced` \| `disabled`. **Read-only** — core has no setter, so a dynamic parameter would silently no-op. |
| `estop_clear_max_age_s` | double | `1.0` | Age beyond which an `engaged: false` message is ignored. `<= 0` disables the check. |

**The staleness check is deliberately asymmetric, and only ever fails safe:**

- **`engaged: true` is never age-checked.** A stale stop is still honoured. Refusing an
  old e-stop because its clock looked wrong is precisely the failure we must not build.
- **`engaged: false` is age-checked.** A clear older than `estop_clear_max_age_s` is
  ignored and logged at WARN. This is what stops a `ros2 bag` replay, or a message
  delayed behind a network hiccup, from silently re-enabling a stopped arm.

Both branches degrade toward "the arm stays stopped", which is the only acceptable
direction for this topic.

## Threading — the one hard constraint

**`/estop`'s subscription gets its own `MutuallyExclusive` callback group.**

`Arbiter::estop()` is deliberately written to latch `estopped_` *before* contending for
`m_`, because a delegated call can hold that mutex for hundreds of milliseconds. Core's
own comment: *"a delegated call may block for hundreds of milliseconds (the streaming
tier's mode settle), so the e-stop latch and its halt both run outside `m_`."*

Today the blocking delegate is a cuRobo plan round-trip on the reentrant group; after the
streaming spec it is `on_stream_open`'s 250 ms mode settle. If `/estop` shared a callback
group with either, we would rebuild in ROS the exact stall core removed in C++. This is a
requirement, not a tuning knob, and it gets its own test.

The node already runs a `MultiThreadedExecutor`, so this costs only a group.

## Data flow

**Acquire → command → release**

1. Client calls `/acquire_control` with `owner_id: "dojo_teleop"`.
2. `ArbitrationServer` → `arb.grant("dojo_teleop")` → `GrantResult{accepted, token, generation}`.
3. `/control_status` publishes (owned, owner_id, generation bumped).

> **`/acquire_control` seizes ownership — it does not queue or fail when the arm is
> already owned.** `grant()` succeeds unless the arm is e-stopped, and when there is an
> incumbent it runs revoke-then-grant: the previous owner is dispossessed and
> `on_halt(kOwnershipRevoked)` fires, settling their in-flight goal with `-9`. Core's
> comment: *"a re-grant is revoke-then-grant, never a silent swap under a moving arm."*
> This is deliberate — the alternative is a silent ownership swap while the arm moves —
> but it means **acquiring control can abort someone else's motion**, and clients must be
> written expecting it. A previous owner detects this as a bumped `generation` on
> `/control_status`, plus a `-9` result on the goal it thought it was running.
4. Client sends any of the four actions with `token` set. `Arbiter::admit()` passes;
   delegates to `Supervisor` exactly as today.
5. Client calls `/release_control` with its token; wrapper verifies it against `status()`,
   then `arb.revoke()`. `revoke()` also delivers `on_halt(kOwnershipRevoked)`.

**E-stop**

1. Anyone publishes `EStop{engaged: true, source, reason}` on `/estop`.
2. `ArbitrationServer` → `arb.estop()`. Latches immediately, delivers
   `on_halt(kEmergencyStop)` without waiting for `m_`, then clears ownership under it.
3. **In-flight ROS goals terminate on their own.** `Supervisor`'s sampler settles both the
   active *and* the queued goal with `result_code::kHalted (-9)` — core's comment:
   *"ACCEPTed already; dropping it orphans the client."* That flows through `GoalRouter`
   → the owning server's `terminal()` → the ROS goal handle aborts with `-9`.
   **No wrapper work is required for goal termination.**
4. `EStop{engaged: false}` → `arb.estop_clear()`, which unlatches and **exits to
   no-owner** — never straight back to owned.

**Consequence worth stating loudly:** engaging the e-stop *itself* destroys ownership
(`owned_ = false; token_ = Token{}`). Every `/estop` cycle silently invalidates all
outstanding tokens; clients must re-acquire. `generation` on `/control_status` is how a
client detects it was dispossessed, which is what makes that topic load-bearing rather
than merely informational.

`estop()` is idempotent (it latches, and deliberately delivers `on_halt` twice), so a
repeating or latched publisher is safe.

## Cancel: the stored-token replay

`Arbiter::on_trajectory_cancel` **is** gated by `admit(c.token)`. But a ROS action cancel
carries no user payload: `action_msgs/srv/CancelGoal` is generated identically for every
action type and its request is one `GoalInfo` (UUID + stamp). There is nowhere to put a
token, and `rclcpp_action` hands `handle_cancel` only the goal handle.

Passing a zero token — which is what the code does today — means **cancel is silently
refused under `arbitration_mode:=enforced`**: the Arbiter rejects it, bumps
`rejected_count`, and the motion keeps running while the client believes it cancelled.

**Fix:** store the token when the goal is accepted and replay it on that goal's cancel.
`Ros2Backend::handles_` and `PlannedMoveServer::goals_` each grow a `Token` field;
`on_trajectory_cancel({id, {}})` becomes `on_trajectory_cancel({id, stored_token})`.

This is a **functional** fix, not an authentication one — see below.

## Deliberate deviations from core's intent

Recorded here so a future reader does not mistake them for oversights.

**1. Cancel is unauthenticated.** Core's `CancelRequest` comment is explicit: *"Cancel had
no struct to carry a token; it needs one, or any stranger can stop your motion."* The ROS
action protocol cannot express this. Worse, `CancelGoal.srv` specifies that *"if the goal
ID is zero and timestamp is zero, cancel all goals"* — so any node on the domain can
cancel everything in flight with one zero-filled call, without knowing any UUID.

We accept this, for two reasons:

- It is a property of the ROS action protocol, not of this design — equally true of the
  node as it exists today.
- **Cancel is a stop-class operation.** It drives the arm to `kPreempted` and holds; it
  never commands new motion. By the rule above, it belongs on the ungated side, same as
  `/estop`. Worst case is nuisance: someone halts a trajectory and leaves the arm mid-path.
- **Authenticating cancel would buy nothing anyway**, because `/acquire_control` is
  itself unauthenticated — see below.

## What arbitration is, and is not

**Arbitration is cooperative coordination, not authorization.** It prevents two
well-behaved clients from commanding the arm at once. It does **not** defend against a
deliberate or buggy actor, and this spec should not be read as claiming otherwise.

The reason is `grant()`: it takes an `owner_id` string, verifies nothing, and always
succeeds unless the arm is e-stopped — seizing ownership from the incumbent if there is
one. So any node on the domain can call `/acquire_control`, receive a valid token, and
command the arm freely. Gating the *command* paths on a token therefore stops accidents
and races, not adversaries.

This is core's intended model — a single trusted orchestrator sequencing cooperating
modules — and it is the right trade for this system. It is recorded here so that nobody
later mistakes the token for a security boundary, and so the choices that follow from it
(open cancel, open e-stop clear) read as consistent rather than as gaps.

### The operational contract

**`/acquire_control` is called by the task orchestrator, and by nothing else.**

This is a convention, not an enforced rule — the driver cannot tell an orchestrator from
anything else, and deliberately does not try. Everything in this tier is calibrated to
that assumption: the participants are known and cooperating, and what we are defending
against is a *mistake* — two modules commanding at once, a stale client resuming after a
handover, a script left running — not an adversary.

A module that acquires control directly, rather than being sequenced by the orchestrator,
is not defeating a security mechanism. It is violating an operational contract, and the
observable symptom is someone else's motion aborting with `-9`.

### Making mistakes loud

Because mistakes are the actual threat model, the seizure path is the one worth
instrumenting. `ArbitrationServer` reads `status()` before delegating to `grant()`, and when it
finds an incumbent it logs at **WARN**, naming both the dispossessed `owner_id` and the
new one.

Nothing is refused — seizure is core's intended behaviour and the orchestrator relies on
it for handover. But an unexpected seizure is almost always the exact class of bug this
tier exists to catch, and it should not be inferable only from a `generation` counter
nobody was watching.

### Future: unknown actors

When this system has to accommodate participants that are not known and trusted, the
answer is **SROS2 / DDS Security** — enclave-based access control over who may reach
these services at the transport layer — not more tokens at this one. That change is
additive: it restricts *who can call*, and leaves the ownership semantics designed here
intact.

**2. `/estop` may be cleared by anyone.** Engaging is fail-safe (anyone may stop the arm);
clearing has the opposite risk profile. Accepted for symmetry and simplicity: the e-stop
destroys ownership, so a spurious clear re-enables an arm that nobody owns and that
cannot be commanded until someone re-acquires.

## Testing

**Unit — `ArbitrationServer` against a fake `ArbitrationSink`** (no robot, no `Supervisor`):

- `/acquire_control` → `grant()` called with the request's `owner_id`; token and
  generation returned verbatim.
- `/release_control` with a **matching** token → `revoke()` called.
- `/release_control` with a **non-matching** token → `revoke()` NOT called, `released=false`.
- `/revoke_control` → `revoke()` called with no token check.
- `/estop {engaged:true}` → `estop()`; `{engaged:false}` → `estop_clear()`.
- **A stale `engaged:false` (stamp older than `estop_clear_max_age_s`) does NOT call
  `estop_clear()`** — the bag-replay guard.
- **A stale `engaged:true` DOES call `estop()`** — the asymmetry, asserted explicitly so
  nobody "tidies" it into a symmetric check later.
- `/release_control` after an `/estop` cycle is refused: the retained token was cleared
  when the stop engaged.
- `/control_status` publishes on change and **not** on every poll; a late subscriber
  receives the current state (`transient_local`).
- Diagnostics level mapping: ERROR when estopped, WARN when unowned in enforced mode,
  else OK.

**Integration — real `Arbiter` + `Supervisor`:**

- Enforced mode: a goal with a granted token is accepted; the same goal with a zero or
  stale token is rejected and `rejected_count` increments.
- Disabled mode: a zero token is admitted (backward-compatibility gate).
- **Cancel in enforced mode succeeds** — the regression test for the stored-token replay.
- **`/estop` settles an in-flight goal with `-9`**, through the real `GoalRouter`.
- **`/acquire_control` while another client owns a moving arm** settles the incumbent's
  goal with `-9`, bumps `generation`, and issues a working token to the new caller —
  the seizure semantics, asserted so the behaviour is a decision rather than a surprise.
- The dispossessed client's subsequent goal, carrying its now-stale token, is rejected.
- A seizure emits the WARN log naming both owners; an uncontested acquire does not.
- `estop_clear` leaves `owned=false` and a bumped `generation`.
- **E-stop latency:** `/estop` is honoured while a slow delegated call holds the arbiter's
  mutex — the test that protects the callback-group requirement.

**Regression:** all 42 existing tests stay green under the default `disabled` mode.

## Backward compatibility

`arbitration_mode` defaults to `disabled`, which costs no safety — `estop()` latches over
*both* modes, and `kDisabled` is the one thing e-stop does not bypass. Under the default,
`send_trajectory.py`, `scripts/abra_e2e_sim.sh`, `make e2e` and the dojo workflows keep
working untouched; enabling arbitration is an opt-in launch argument.

The `.action` files change, so all clients must be **rebuilt** even though their behaviour
is unchanged.

## Out of scope (future specs — NOT this PR)

- **Streaming tier** — setpoint topics, `StreamSink`, session open/close. Spec 2, which
  builds directly on this one: every setpoint message carries a token minted here.

    **Decision: spec 2 designs all 5 setpoint kinds**, assuming `JointVelocityMode` lands
    in core per its streaming-setpoints design. The skeleton is already in — the
    `ControlModeKind::kVelocity` enum, the stubbed `on_setpoint_joint_velocity` /
    `on_setpoint_twist`, and the `pair_supported()` rows that currently refuse them — so
    the ROS surface can be designed against the finished shape and the rows flip on when
    the mode lands.

    **Tracked risk, not a blocker.** Core has not yet written the mode because it does not
    know the arm honours velocity commands: no `ControlMode` has ever used that path, and
    `SimTransport` "is a static echo with no plant", so it cannot be settled in sim.
    Kinova's own `ros2_kortex` computes a velocity command and then declines to send it
    ("Velocity command interface not implemented properly in the kortex api").
    `apps/velocity_probe` answers this on the real arm and **has no recorded result** as of
    this writing. If it returns `IGNORES`, core notes the twist path "needs redesigning"
    toward a torque-domain alternative (`tau = g(q) + Kd*(qd_des - qd)`) — which would
    change the *mode*, not the ROS topics, so spec 2's surface should survive it.
- **`set_gains` / `query_state` services** — both stubbed in the `Supervisor`; both now
  carry tokens. `query_state` is never gated by the Arbiter regardless.
- **Ownership lease / heartbeat.** Explicitly declined: `/revoke_control` is the recovery
  path for a crashed owner.
- **SROS2 / DDS Security.** The answer for when the domain contains unknown actors rather
  than known, cooperating ones — transport-level access control over who may call these
  services at all, including cancel and `/acquire_control`. Deferred as whole-domain
  infrastructure (keystores, enclaves, certificates), and additive when it comes: it
  changes who may call, not what the calls mean.
- **`diagnostic_aggregator` configuration** (`/diagnostics_agg`) — we publish REP 107
  correctly; aggregation is a deployment concern.
