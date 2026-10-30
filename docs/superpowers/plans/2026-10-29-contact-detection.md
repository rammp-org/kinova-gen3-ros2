# Unexpected-Contact Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the Gen3 on unexpected contact: a momentum-observer residual per joint, checked every cycle in the driver's RT loop, switches the arm to a floating mode and latches until `/clear_faults`.

**Architecture:** Core (`kinova-gen3-driver`) gets `Dynamics::coriolis`, a `MomentumObserver`, a `ContactMonitor` (observer + thresholds + debounce + latch, RT-safe, status out through a `Seqlock`), a `FloatMode` `ControlMode`, and an `RtExecutor` hook that adopts float on a trip and holds it while latched. The Supervisor owns profiles, the token-gated threshold command, the `HaltReason::kContact` halt, refusals and the clear. The ROS node gets parameters, a service, a status topic, diagnostics and the `/clear_faults` extension. Interfaces get one srv and one msg. (Updated 2026-10-30, Swapnil's ruling: the status topic and its msg are removed; the diagnostics carry the state, and an opt-in debug topic carries `r`. See Task 8 and Task 9.)

**Tech Stack:** C++17, Eigen, Pinocchio 2.7 (cmeel `pin==2.7.0`), gtest; ROS 2 Humble, rclcpp, diagnostic_updater; rosidl.

**Spec:** `docs/superpowers/specs/2026-10-29-contact-detection-design.md` (this repo). Read it first.

## Global Constraints

- RT rule (driver CLAUDE.md): nothing in `ControlMode::compute`, `ContactMonitor::step` or the executor cycle allocates, locks or blocks. Every new RT object preallocates in its constructor.
- Fail loud at startup: invalid contact config throws from the constructor naming the field; the node exits non-zero.
- Refuse loudly, always: every invalid threshold request is refused with a message naming the joint and the limit. Nothing is clamped silently.
- Torque sign: `JointFeedback::tau` is command-convention (≈ +g(q) at free hold). External torque at rest = `g(q) − tau`.
- Observer: `r = K (p − p0 − ∫(τ + Cᵀq̇ − g + r) dt)`, `p = M(q) q̇`. Default `K = 25` rad/s. Settling window `5/K` s. Debounce default 5 consecutive good cycles.
- Re-init the observer only at each `RtExecutor::run` start and on a clear. Never on a control-mode change.
- The contact latch survives everything except a clear (ArmLink faults, reconnects, e-stop, ownership changes). While latched the executor never leaves float.
- Result code `kContactStop = -11`. Halt string `"halted: unexpected contact"`.
- Float: `τ = g(q) − D q̇ − K_s (q − q_a)` with a dragged anchor, band `δ` = 2°. Starting `K_s` 28.6 N·m/rad on j1–j4, 8.6 N·m/rad on j5–j7 (so `K_s·δ` ≈ 1.0 / 0.3 N·m); `D` 5.0 N·m·s/rad on j1–j4, 1.5 on j5–j7; torque limit `[39,39,39,39,9,9,9]`. These are starting values and will be tuned on the arm.
- Thresholds (N·m, j1..j7), from the 2026-10-30 push tests and still tuned on the arm with Swapnil:
  sensitive `[8,8,6,6,2,2,2]`, default `[16,16,12,12,4,4,4]`, loose `[24,24,18,18,6,6,6]`, ceiling `[30,30,22,22,7,7,7]`.
- Detection modes: `enforce` (default), `observe`, `off`. Not switchable at runtime.
- Branches: driver `SwapnilPande/contact-detection` off `origin/dev`; interfaces `SwapnilPande/contact-detection` off `origin/dev`; this repo `SwapnilPande/safety-contact-detection`. Stage explicit paths; never `git add -A`; never commit `.hil.yml`. Don't merge, tag or release.
- No arm motion without Swapnil confirming each batch (what, speed, position vs impedance). abra and the arm are shared: ask before using them.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

**Driver build/test** (no host cmake on muk; see memory `kinova-driver-local-build`), from the driver checkout:

```sh
docker run --rm -v "$PWD":/src -v "$SCRATCH":/sp -w /src kinova-gen3-ros2:amd64-probe bash -c '
  python3 -m pip install -q "pin==2.7.0"
  PREFIX="$(python3 -c "import sysconfig,os; print(os.path.join(sysconfig.get_paths()[\"purelib\"],\"cmeel.prefix\"))")"
  export LD_LIBRARY_PATH="$PREFIX/lib:$LD_LIBRARY_PATH"
  cmake -S /src -B /sp/bld -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"
  cmake --build /sp/bld -j"$(nproc)" && /sp/bld/unit_tests --gtest_filter="$FILTER"'
```

Below, "run driver tests `<filter>`" means this command with `FILTER=<filter>`. New test files are appended to the single `add_executable(unit_tests ...)` list in the driver's top `CMakeLists.txt`, and new sources to the `kinova_lowlevel` library sources.

**Node build/test:** `make build CORE_REF=SwapnilPande/contact-detection` plus `--build-arg INTERFACES_REF=SwapnilPande/contact-detection` (Dockerfile `ARG INTERFACES_REF`; add it to the Makefile the way `CORE_REF` is passed if it isn't there), then in `make shell`: `colcon test --packages-select kinova_gen3_ros2 --ctest-args -R <test> && colcon test-result --verbose`.

## Review Focus

1. **Comms loss or an ArmLink fault while latched.** Expected: on the next `RtExecutor::run`, the arm comes back in float, still latched, with goals refused. It must not come back in the Supervisor's position hold. Test: Task 5 `ContactExecutor.LatchSurvivesRunRestart`.
2. **A mode request racing the trip** (goal promotion or stream open in the same millisecond). Expected: the executor stays in float, and the pending request is adopted only after the clear. Test: Task 5 `ContactExecutor.RequestWhileLatchedDeferredUntilClear`.
3. **Clearing while the person still presses.** Expected: blind for `5/K`, then a new trip. Test: Task 4 `ContactMonitor.ClearWhilePressedRetripsAfterSettle`.
4. **Non-finite torque or velocity in feedback** (a garbled frame or a fault). Expected: the sample is skipped, the observer integral isn't poisoned, there's no trip on NaN, and the count is reported. Test: Task 4 `ContactMonitor.NonFiniteSampleSkipped`.
5. **Bad threshold requests**: wrong length, NaN, ≤0, above the ceiling, unknown profile, or no token. Expected: refused, with the joint and the limit in the message; active thresholds unchanged. Tests: Task 6 `ContactThresholds.*` and Task 9 `ContactServer.RefusesBadRequests`.

---

## Part A: driver (`rammp-org/kinova-gen3-driver`, branch off `origin/dev`)

### Task 1: `Dynamics::coriolis`

**Files:**
- Modify: `include/kinova_lowlevel/dynamics.h`, `src/dynamics.cpp`
- Test: `tests/dynamics_test.cpp`

**Interfaces:**
- Produces: `void Dynamics::coriolis(const JointVec& q, const JointVec& qd, JointMat& C_out);`, RT-safe with no allocation after the constructor. It returns the Christoffel-consistent `C(q, q̇)` (Pinocchio `computeCoriolisMatrix`), so `Ṁ = C + Cᵀ`.

- [x] **Step 1: Write failing tests** in `tests/dynamics_test.cpp`:

```cpp
TEST(Dynamics, CoriolisTimesQdMatchesNonlinearMinusGravity) {
  // For 20 random (q, qd) in joint limits / |qd|<=1.5: C*qd == rnea(q,qd,0) - g(q).
  // rnea(q,qd,0) computed as mass-free reference: use pinocchio directly in the test,
  // or (M*0 + C*qd + g) consistency via finite difference of p = M qd (below).
  EXPECT_LT((C * qd - (nle - g)).cwiseAbs().maxCoeff(), 1e-9);
}
TEST(Dynamics, MdotMinusTwoCIsSkewSymmetric) {
  // Mdot by central difference along qd: (M(q + h qd) - M(q - h qd)) / 2h, h = 1e-6.
  // x^T (Mdot - 2C) x ~ 0 for random x.
  EXPECT_NEAR(x.dot((Mdot - 2 * C) * x), 0.0, 1e-5);
}
```

The first test gets `nle` from `pinocchio::nonLinearEffects` on the test's own Pinocchio model built from `URDF_PATH`, reduced the way `Dynamics` reduces it. If that is awkward, drop that test and keep only the skew-symmetry test plus `C(q, 0) == 0`.

- [x] **Step 2: Run** driver tests `Dynamics.Coriolis*:Dynamics.MdotMinusTwoC*`. Expected: compile failure, because `coriolis` is not declared.
- [x] **Step 3: Implement** with `pinocchio::computeCoriolisMatrix(model, data, qcfg, qd)` after `pack(q)`, then copy `data.C` into `C_out`. Add a header comment in the style of `mass_matrix`.
- [x] **Step 4: Run** the same filter, then all of `Dynamics.*`. Expected: PASS.
- [x] **Step 5: Commit** `include/kinova_lowlevel/dynamics.h src/dynamics.cpp tests/dynamics_test.cpp` with message `feat(dynamics): coriolis matrix for the momentum observer`.

### Task 2: `MomentumObserver`

**Files:**
- Create: `include/kinova_lowlevel/momentum_observer.h`, `src/momentum_observer.cpp`
- Test: `tests/momentum_observer_test.cpp`

**Interfaces:**
- Consumes: `Dynamics::mass_matrix`, `Dynamics::coriolis`, `Dynamics::gravity`.
- Produces:

```cpp
class MomentumObserver {
 public:
  MomentumObserver(Dynamics& dyn, double gain);   // throws std::invalid_argument unless gain finite and > 0
  void reset() noexcept;                            // next step() seeds p0 and returns zeros
  // RT-safe. tau command-convention. Returns r (external joint torque estimate).
  const JointVec& step(const JointVec& q, const JointVec& qd, const JointVec& tau, double dt_s) noexcept;
  const JointVec& residual() const noexcept;
  double gain() const noexcept;
  double time_since_reset() const noexcept;        // sum of dt since reset
};
```

The header comment carries the derivation and the sign convention, condensed from the teammate's `observer.py` (`feat/momentum-observer-demo:kinova_gen3_ros2/test/momentum_observer/observer.py`), with credit to it.

- [x] **Step 1: Write failing tests:**

```cpp
TEST(MomentumObserver, RestConvergesToInjectedTorque) {
  // q fixed at a non-singular pose, qd = 0, tau = g(q) - tau_e, tau_e = (0, 3, 0, 2, 0, 0.5, 0).
  // 1 kHz, K = 25. After 5/K s: |r - tau_e| < 0.01 * |tau_e| per joint.
}
TEST(MomentumObserver, MotionWithoutContactStaysNearZero) {
  // q(t) = q0 + A sin(w t), A = 0.4 rad on j1-j4 and 0.6 on j5-j7, w = 2 rad/s, 4 s at 1 kHz.
  // tau = M qdd + C qd + g (no external torque).
  // After settling: max |r| < 0.05 N·m, while max |g - tau| on j2 > 1.0 N·m
  // (shows that the inertia the quasi-static residual carries is rejected).
}
TEST(MomentumObserver, PushDuringMotionRecovered) {
  // Same trajectory, tau_e = 4 N·m on j4 from t = 2.0 to 2.5 s.
  // At t = 2.0 + 5/K: |r_4 - 4| < 0.2 and other joints < 0.1.
}
TEST(MomentumObserver, RejectsBadGain) { EXPECT_THROW(MomentumObserver(dyn, 0.0), std::invalid_argument); }
```

- [x] **Step 2: Run** driver tests `MomentumObserver.*`. Expected: compile failure.
- [x] **Step 3: Implement.** Preallocate `M_`, `C_`, `g_`, `p_`, `p0_`, `integral_` and `r_` as members. The integration is explicit Euler, as in the Python reference.
- [x] **Step 4: Run** `MomentumObserver.*`. Expected: PASS.
- [x] **Step 5: Commit** the three files plus the CMake list edits, with message `feat: momentum observer for external joint torque`.

### Task 3: `FloatMode`

**Files:**
- Create: `include/kinova_lowlevel/float_mode.h`, `src/float_mode.cpp`
- Test: `tests/float_mode_test.cpp`; add a steady-state case to `tests/rt_safety_test.cpp`
- Docs: `docs/reference/api.md` under `## Control modes` (new `### FloatMode`), `docs/guide/control-modes.md`

**Interfaces:**
- Produces:

```cpp
struct FloatParams {
  JointVec damping = (JointVec() << 5, 5, 5, 5, 1.5, 1.5, 1.5).finished();          // N·m·s/rad
  JointVec spring  = (JointVec() << 28.6, 28.6, 28.6, 28.6, 8.6, 8.6, 8.6).finished(); // N·m/rad
  double drag_band_rad = 2.0 * kDeg2Rad;
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
};
class FloatMode : public ControlMode {
 public:
  FloatMode(Dynamics& dyn, FloatParams p = {});   // throws std::invalid_argument: non-finite or negative gains, band <= 0, limit <= 0
  ActuatorModes required_modes() const override;  // all kTorque
  void on_enter(const JointFeedback& fb) override; // anchor = fb.q
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}
  const JointVec& anchor() const noexcept;
};
```

The anchor drag, per joint: `e = q − q_a`; if `|e| > δ`, set `q_a = q − sign(e)·δ`. Torque is `g − D·qd − K_s·(q − q_a)`, clamped to `±torque_limit`. `out.mode = kTorque`.

- [x] **Step 1: Write failing tests:**

```cpp
TEST(FloatMode, AtRestCommandsGravityOnly)          // q = anchor, qd = 0 -> out.torque == g(q)
TEST(FloatMode, SpringForceBoundedByBand)          // push q 10° past anchor on j2: |tau - g| <= K_s*δ + 1e-9, and anchor moved to q - δ
TEST(FloatMode, StaysWhereLeft)                    // after a 10° drag, return q to the final pose: anchor within δ of q, no pull toward the trip pose
TEST(FloatMode, DampingOpposesVelocity)            // qd = +1 on j4 -> tau_4 - g_4 == -D_4 (anchor at q)
TEST(FloatMode, ClampsToTorqueLimit)
TEST(FloatMode, RejectsBadParams)
TEST(RtSafety, FloatModeNoMajorFaultsSteadyState)  // reuse expect_*_steady_state pattern with FloatMode
```

- [x] **Step 2: Run** driver tests `FloatMode.*:RtSafety.FloatMode*`. Expected: compile failure.
- [x] **Step 3: Implement.** Write the docs sections: what float is for (spec §3), the law, and the parameter meanings.
- [x] **Step 4: Run** `FloatMode.*:RtSafety.FloatMode*`. Expected: PASS.
- [x] **Step 5: Commit** the files above, with message `feat: FloatMode, the contact-stop control law`.

### Task 4: `ContactMonitor`

**Files:**
- Create: `include/kinova_lowlevel/contact_monitor.h`, `src/contact_monitor.cpp`
- Test: `tests/contact_monitor_test.cpp`

**Interfaces:**
- Consumes: `MomentumObserver` (Task 2), `Seqlock` from `feedback_tap.h`.
- Produces:

```cpp
enum class ContactDetection : uint8_t { kObserve, kEnforce };  // "off" = no monitor attached (node, Task 9)
enum class ContactPhase : uint8_t { kSettling, kArmed, kLatched };
struct ContactTrip {
  uint64_t cycle = 0;        // monitor cycle count at the trip
  int joint = -1;            // 0-based, the first joint to reach the debounce count
  JointVec r, theta, q, qd;
};
struct ContactStatus {
  ContactDetection detection = ContactDetection::kEnforce;
  ContactPhase phase = ContactPhase::kSettling;
  JointVec r = JointVec::Zero();
  JointVec theta = JointVec::Zero();
  uint64_t would_trips = 0;   // crossings that reached the debounce count (both modes)
  uint64_t bad_samples = 0;   // non-finite q/qd/tau skipped
  bool has_trip = false;
  ContactTrip last{};
};
struct ContactMonitorConfig {
  ContactDetection detection = ContactDetection::kEnforce;
  double gain = 25.0;
  int debounce_cycles = 5;
  JointVec initial_theta;    // must pass validate_contact_thresholds against `ceiling`
  JointVec ceiling;
};
struct ThresholdCheck { bool ok = false; std::string message; };
// Non-RT, pure. Refuses: non-finite, <= 0, or > ceiling[i]; message names the joint (1-based) and the limit.
ThresholdCheck validate_contact_thresholds(const JointVec& theta, const JointVec& ceiling);

class ContactMonitor {
 public:
  ContactMonitor(Dynamics& dyn, ContactMonitorConfig cfg);  // throws std::invalid_argument (bad gain, debounce < 1, invalid thresholds or ceiling)
  // Non-RT, single writer (the Supervisor). Caller has validated.
  void set_thresholds(const JointVec& theta) noexcept;
  void request_clear() noexcept;            // consumed by the next step()
  bool latched() const noexcept;
  bool status(ContactStatus& out) const noexcept;   // Seqlock load; false on a torn read
  const JointVec& ceiling() const noexcept;
  ContactDetection detection() const noexcept;
  // RT thread only.
  void on_run_start() noexcept;            // observer reset -> kSettling unless latched; latch kept
  // Returns true while latched (enforce only; observe never latches).
  bool step(const JointFeedback& fb, double dt_s, bool missed) noexcept;
};
```

Semantics, in step order:
1. Consume a pending clear: unlatch, reset the observer, phase becomes `kSettling`.
2. If `missed`, or any of q/qd/tau is non-finite: increment `bad_samples` on non-finite input, then publish and return. The observer is not stepped and the debounce counters are unchanged. For a missed frame, the next good sample integrates with `dt` accumulated across the gap. (Updated 2026-10-30 after review: `dt_s` is the measured time between feedback reads, and a good step integrates at most `1/K`; see the spec.)
3. Step the observer. While `time_since_reset < 5/K`, the phase is `kSettling` and the debounce counters stay zero.
4. Per joint, count consecutive cycles with `|r_i| > θ_i` (reset to 0 when not over). The first joint to reach `debounce_cycles` increments `would_trips` and records `last`. In enforce it latches, which wins over everything until a clear. In observe it re-arms its counter.
5. Publish `ContactStatus` to the Seqlock every cycle.

Thresholds are handed to the RT thread through a two-buffer + atomic index, the same pattern as `JointTorqueMode::set_torque`.

- [x] **Step 1: Write failing tests** (feed `JointFeedback` directly; use the Task 2 rest/motion fixtures moved into `tests/fixtures.h`):

```cpp
TEST(ContactMonitor, NoTripDuringSettling)                 // large tau_e from t=0 -> phase kSettling, no latch until 5/K
TEST(ContactMonitor, EnforceLatchesOnSustainedPush)        // tau_e 6 N·m j2 > θ 4 -> latched within 5/K + debounce; last.joint == 1
TEST(ContactMonitor, DebounceRejectsShortSpike)            // 3-cycle spike above θ with debounce 5 -> no latch
TEST(ContactMonitor, ObserveCountsButNeverLatches)         // would_trips == 1, latched() == false, step() returns false
TEST(ContactMonitor, LatchHoldsWithoutClear)               // tau_e removed -> still latched after 10 s
TEST(ContactMonitor, ClearWhilePressedRetripsAfterSettle)  // Review Focus 3
TEST(ContactMonitor, NonFiniteSampleSkipped)               // Review Focus 4: NaN tau -> bad_samples 1, r finite, no latch
TEST(ContactMonitor, MissedFramesDoNotStepObserver)
TEST(ContactMonitor, MotionWithoutContactNoTrip)           // Task 2 trajectory, default thresholds -> would_trips == 0
TEST(ContactMonitor, ThresholdChangeTakesEffectNextCycle)
TEST(ContactThresholds, ValidateRefusesAndNamesJoint)      // NaN / 0 / -1 / ceiling+0.1 on joint 5 -> !ok, message contains "joint 5" and the limit
TEST(ContactMonitor, CtorRejectsBadConfig)
TEST(RtSafety, ContactMonitorNoMajorFaultsSteadyState)
```

- [x] **Step 2: Run** driver tests `ContactMonitor.*:ContactThresholds.*:RtSafety.ContactMonitor*`. Expected: compile failure.
- [x] **Step 3: Implement.**
- [x] **Step 4: Run** the same filter. Expected: PASS.
- [x] **Step 5: Commit** with message `feat: ContactMonitor, per-joint contact detection with latch`.

### Task 5: Executor hook, plus sim external torque

**Files:**
- Modify: `include/kinova_lowlevel/rt_executor.h`, `src/rt_executor.cpp`, `include/kinova_lowlevel/telemetry.h` (flag bit), `include/kinova_lowlevel/sim_transport.h`, `src/sim_transport.cpp`
- Test: `tests/contact_executor_test.cpp`
- Docs: `docs/reference/api.md` `## RtExecutor` (attach_contact, latch semantics, flag bit 3)

**Interfaces:**
- Produces:
  - `void RtExecutor::attach_contact(ContactMonitor& monitor, ControlMode& on_contact) noexcept;` Call it before `run()`; both arguments must outlive the executor.
  - `CycleSample::flags` bit3 (`8u`) means the contact latch is set this cycle.
  - `void SimTransport::set_external_torque(const JointVec& tau_ext);` Reported `tau` becomes `(what it reports today) − tau_ext`. It is safe to call while running, guarded like `set_arm_fault`.

Loop changes, with step numbers from `rt_executor.cpp`:
- At `run()` start, call `monitor.on_run_start()`.
- Step 3, adopt: if latched and `active != on_contact`, adopt `on_contact` through the existing adopt path (`on_exit`, `set_actuator_modes`, `on_enter(fb)`), and **do not touch `requested_`**. If latched and already in float, skip adoption entirely: `requested_` stays pending. Otherwise, run the existing logic.
- After step 4 (exchange): `const bool latched = monitor.step(fb, dt_s, missed);`.
- Trip cycle: if `latched && active != on_contact`:
  - If `on_contact.required_modes() == active->required_modes()`, switch now (`on_exit`, `on_enter(fb)`, `active = &on_contact`).
  - Otherwise freeze: `cmd.position = fb.q; cmd.velocity.setZero();`, skip `compute` this cycle, and step 3 adopts float at the next boundary.
- No monitor attached: behaviour is byte-for-byte unchanged.

- [x] **Step 1: Write failing tests**, using `SimTransport` with initial `tau = g(q)` and real modes on `Dynamics(URDF_PATH)`:

```cpp
TEST(ContactExecutor, PositionModeTripAdoptsFloat)                   // pos mode, after settle set_external_torque(6 on j2) -> within 50 cycles active is float: last_command().mode == kTorque
TEST(ContactExecutor, TorqueModeTripSwitchesSameCycle)               // tau mode -> the cycle that latches already commands float's torque
TEST(ContactExecutor, RequestWhileLatchedDeferredUntilClear)         // Review Focus 2: request_mode(&pos) while latched -> stays float; request_clear() -> pos adopted
TEST(ContactExecutor, LatchSurvivesRunRestart)                       // Review Focus 1: latch, stop run(), run() again -> first adopted mode is float
TEST(ContactExecutor, ObserveNeverChangesCommand)                    // observe: same last_command() sequence with and without set_external_torque
TEST(ContactExecutor, NoMonitorUnchanged)                            // existing behaviour without attach_contact
TEST(ContactExecutor, LatchFlagInTelemetry)                          // ring sample has bit3 after the trip
```

- [x] **Step 2: Run** driver tests `ContactExecutor.*`. Expected: compile failure.
- [x] **Step 3: Implement.**
- [x] **Step 4: Run** `ContactExecutor.*:RtSafety*-RtSafety.NanosleepPacingProducesSamples:RtExecutor*`. Expected: PASS, and the existing executor tests still pass.
- [x] **Step 5: Commit** with message `feat(executor): adopt FloatMode on a contact trip and hold it while latched`.

### Task 6: Supervisor, Arbiter and value types

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h`, `include/kinova_lowlevel/interface/ports.h`, `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`, `include/kinova_lowlevel/interface/arbiter.h`, `src/interface/arbiter.cpp`
- Create: `include/kinova_lowlevel/interface/contact.h`, `src/interface/contact.cpp`
- Test: `tests/interface/contact_supervisor_test.cpp`; also keep `tests/interface/fake_backend.h` compiling
- Docs: `docs/reference/api.md` `## Arbitration` (HaltReason list: add `kContact`, and add the missing `kArmUnavailable`, because the list is touched anyway), `### CommandSink::on_halt`, `### Gating`, `### SupervisorDeps`, result codes. Also `CHANGELOG.md`.

**Interfaces:**
- Produces in `value_types.h`:
  - `HaltReason::kContact`; `result_code::kContactStop = -11`
  - `enum class ContactProfile { kDefault, kSensitive, kLoose, kCustom };` (0..3, the wire's `PROFILE_*` values)
  - `struct ContactThresholdsRequest { ContactProfile profile = ContactProfile::kDefault; JointVec custom = JointVec::Zero(); Token token{}; };`
  - `struct ContactThresholdsResult { bool accepted = false; std::string message; ContactProfile profile = ContactProfile::kDefault; JointVec theta = JointVec::Zero(); };`
  - `struct ContactState { bool enabled = false; ContactProfile profile = ContactProfile::kDefault; ContactStatus status{}; };`
- Produces in `interface/contact.h`:
  - `struct ContactProfiles { JointVec sensitive, normal, loose, ceiling; };`
  - `ThresholdCheck validate_profiles(const ContactProfiles&);` checks every named profile against the ceiling.
  - `ThresholdCheck resolve_contact(const ContactThresholdsRequest&, const ContactProfiles&, JointVec& theta_out);`
- In `ports.h`, on `CommandSink`, both virtual with defaults, so existing fakes keep compiling:
  - `virtual ContactThresholdsResult on_set_contact_thresholds(const ContactThresholdsRequest&)` defaults to `{false, "contact detection not available"}`.
  - `virtual ContactState on_query_contact()` defaults to `{}`.
- `SupervisorDeps`: `ContactMonitor* contact = nullptr;` (optional, like `grip`) and `ContactProfiles contact_profiles{};`. When `contact` is set, the constructor throws unless `validate_profiles` passes and the profiles' `ceiling` equals `contact->ceiling()`.
- `Supervisor`: overrides of both methods, plus `bool clear_contact();`. It returns true if a latch was set; on true it requests `pos_`, holding at the measured q.
- `Arbiter::on_set_contact_thresholds` uses the `admit(token)` pattern (`++rejected_`, message `"not authorized"`). `on_query_contact` is not gated.

Behaviour:
- The sampler watches `contact_->latched()` and calls `on_halt(HaltReason::kContact)` once per rising edge.
- The sampler's halt settlement uses `kContactStop` for `kContact` and `kHalted` for everything else.
- While latched:
  - `on_trajectory_goal` rejects.
  - `on_trajectory_accepted` settles `-11`.
  - `on_stream_open` returns `{false, kContactStop, "stopped on unexpected contact; call /clear_faults"}`.
  - `on_gripper_setpoint` drops the setpoint.
  - `on_set_contact_thresholds` still works.
- `halt_reason_string(kContact)` returns `"halted: unexpected contact"`.

- [x] **Step 1: Write failing tests**, with `FakeBackend`, a real `ContactMonitor` and `SimTransport` as in Task 5:

```cpp
TEST(ContactSupervisor, InFlightGoalSettlesContactStop)    // error_code == -11, error_string == "halted: unexpected contact"
TEST(ContactSupervisor, RefusesWhileLatched)               // goal rejected, stream open {false, -11, ...}
TEST(ContactSupervisor, ClearRequestsPositionHold)         // clear_contact() == true -> after unlatch, active mode is pos; second call false
TEST(ContactSupervisor, LatchSurvivesLinkOutage)           // latch, on_link_state(kConnecting) then (kRunning) -> still latched, executor in float
TEST(ContactThresholds, NamedProfileApplied)
TEST(ContactThresholds, CustomAboveCeilingRefused)         // Review Focus 5: message names joint + ceiling; active θ unchanged
TEST(ContactThresholds, CustomNonFiniteRefused)
TEST(ContactThresholds, UnauthorizedRefusedByArbiter)
TEST(ContactThresholds, CtorRejectsProfilesAboveCeiling)
```

- [x] **Step 2: Run** driver tests `ContactSupervisor.*:ContactThresholds.*`. Expected: compile failure.
- [x] **Step 3: Implement.** Update the docs and `CHANGELOG.md`.
- [x] **Step 4: Run** the full suite (`FILTER=*`). Expected: PASS.
- [x] **Step 5: Commit** with message `feat(interface): contact stop halt, threshold profiles, clear`.

### Task 7: Replay tool and Python parity

**Files:**
- Create: `apps/contact_replay.cpp` (registered in CMake like the other apps), `tools/contact/parity.py`, `docs/guide/contact-detection.md` (tuning procedure, spec §4 steps 1–3, and the bench steps from Part D)

**Interfaces:**
- `contact_replay --urdf F --ee-frame E --csv in.csv --out r.csv [--gain 25] [--debounce 5] [--thresholds a,b,c,d,e,f,g]`:
  - Input columns: `t,q1..q7,qd1..qd7,tau1..tau7`, the teammate's recorder format. A `--tau-sign -1` flag handles raw core v1.2.0 recordings.
  - Output: `t,r1..r7`.
  - Stdout: per-joint motion RMS and peak, plus the would-be trips at the given thresholds.
- `tools/contact/parity.py --observer-py PATH --csv in.csv --urdf F --cpp r.csv`: imports the teammate's `observer.py` from `PATH`, runs `run_observer` at the same gain, and prints the max |Δr| per joint. Exits 1 if any joint is over 0.05 N·m after the first `5/K` s.

- [x] **Step 1: Generate a fixture.** Write a CSV with the Task 2 trajectory and push to `tests/data/contact_synthetic.csv`. Write a gtest `ContactReplay.SyntheticPushFound` that runs the replay's core function, which is factored as `ReplayStats replay(const ReplayInput&, const ReplayOptions&)` in the app's own translation unit, compiled into `unit_tests`. It asserts one would-be trip on j4.
- [x] **Step 2: Run** driver tests `ContactReplay.*`. Expected: FAIL.
- [x] **Step 3: Implement** the app and `parity.py`. Run `parity.py` locally against the synthetic CSV, with `observer.py` taken from `git show origin/feat/momentum-observer-demo:kinova_gen3_ros2/test/momentum_observer/observer.py` written into the scratchpad. Expected: exit 0.
- [x] **Step 4: Run** `ContactReplay.*`. Expected: PASS.
- [x] **Step 5: Commit** with message `feat(apps): contact_replay and parity check against the Python observer`.

Then open the driver PR to `dev`. Its body goes through the `unslop` skill and links #24, rammp-deployments#88, #102, the spec, and the teammate's branch.

## Part B: interfaces (`rammp-org/rammp-interfaces-ros2`, branch off `origin/dev`)

### Task 8: `SetContactThresholds.srv`, `ContactMonitorState.msg`, result-code comments

**Files:**
- Create: `rammp_arm_interfaces/srv/SetContactThresholds.srv`, `rammp_arm_interfaces/msg/ContactMonitorState.msg`
- Modify: `rammp_arm_interfaces/CMakeLists.txt` (rosidl list), `rammp_arm_interfaces/action/{ExecuteJointTrajectory,GoToEEPose,GoToJointConfig,GoToPreset}.action` (add `CONTACT_STOP=-11` to the error_code comment), and any `GoTo*s.action` present on `dev`

`SetContactThresholds.srv`:
```
uint8 PROFILE_DEFAULT=0
uint8 PROFILE_SENSITIVE=1
uint8 PROFILE_LOOSE=2
uint8 PROFILE_CUSTOM=3
uint8 profile
float64[] custom     # exactly 7 values (N·m, joints 1..7) when profile == PROFILE_CUSTOM; empty otherwise
uint8[16] token      # arbitration capability from /acquire_control
---
bool accepted
string message       # why not, when accepted is false
uint8 profile        # active after the call
float64[7] thresholds
```

`ContactMonitorState.msg`:
```
std_msgs/Header header
uint8 DETECTION_OFF=0
uint8 DETECTION_OBSERVE=1
uint8 DETECTION_ENFORCE=2
uint8 detection
uint8 PHASE_SETTLING=0
uint8 PHASE_ARMED=1
uint8 PHASE_LATCHED=2
uint8 phase
uint8 profile                 # SetContactThresholds.PROFILE_*
float64[7] residual           # N·m, observer estimate of external joint torque
float64[7] thresholds
uint64 would_trips
uint64 bad_samples
bool has_trip
int32 trip_joint              # 1-based
float64[7] trip_residual
float64[7] trip_thresholds
float64[7] trip_q
float64[7] trip_qd
```

- [x] **Step 1:** Add the files and the CMake entries.
- [x] **Step 2: Verify** with `colcon build --packages-select rammp_arm_interfaces`, then `ros2 interface show rammp_arm_interfaces/srv/SetContactThresholds`. Run them in the repo's container or CI image. Expected: the build succeeds and the interface prints.
- [x] **Step 3: Commit** with message `feat(arm): SetContactThresholds, ContactMonitorState, CONTACT_STOP result code`, and open a PR (body through `unslop`).

(Updated 2026-10-30, Swapnil's ruling: `ContactMonitorState.msg` is removed before release. It duplicated the node's `Contact detection` diagnostics, and a published type is hard to remove once clients use it. `SetContactThresholds.srv` and `CONTACT_STOP=-11` stay.)

## Part C: ROS node (this repo, `SwapnilPande/safety-contact-detection`)

### Task 9: Parameters, wiring, service, topic, diagnostics, `/clear_faults`

**Files:**
- Create: `kinova_gen3_ros2/include/kinova_gen3_ros2/contact_server.h`, `kinova_gen3_ros2/src/contact_server.cpp`, `kinova_gen3_ros2/include/kinova_gen3_ros2/contact_params.h`, `kinova_gen3_ros2/src/contact_params.cpp`
- Modify: `kinova_gen3_ros2/src/bringup_node.cpp` (after the modes at ~l.225–237, before `ArmLink` at ~l.350), `kinova_gen3_ros2/src/link_server.cpp` + header (`on_clear`), `kinova_gen3_ros2/src/ros2_backend.cpp` (`handle_goal` pre-check warn), `kinova_gen3_ros2/CMakeLists.txt` (a `contact_server` library like `link_server`, plus tests), the gripper action server (reject new goals while `on_query_contact().status.phase == kLatched`)
- Test: `kinova_gen3_ros2/test/contact_server_test.cpp`, `kinova_gen3_ros2/test/contact_params_test.cpp`, extend `kinova_gen3_ros2/test/link_server_test.cpp`
- Docs: `docs/interface.md` (service, topic, result `-11`, parameters, and a "Contact stops" section: what trips it, float, latch, clear, observe/off, the deliberate disable), `docs/on-robot-runbook.md` (what to do after a contact stop)

**Interfaces:**
- `contact_params.h`:
  - `struct ContactNodeConfig { std::string detection; kinova::ContactMonitorConfig monitor; kinova::interface::ContactProfiles profiles; kinova::FloatParams floating; };`
  - `ContactNodeConfig declare_contact_params(rclcpp::Node&);` throws `std::invalid_argument` naming the parameter.
  - Read-only parameters, with defaults from Global Constraints:
    - `contact_detection` (`"enforce"|"observe"|"off"`)
    - `contact_gain`, `contact_debounce_cycles`
    - `contact_thresholds_sensitive|default|loose|ceiling` (double[7])
    - `float_damping`, `float_spring` (double[7]), `float_drag_band_deg`
- `contact_server.h`: `ContactServer(rclcpp::Node::SharedPtr, kinova::interface::CommandSink& gated /*the Arbiter*/, std::string detection);`
  - Service `/set_contact_thresholds` maps the request to `ContactThresholdsRequest` and refuses wrong-length `custom` itself, naming the expected 7.
  - Topic `/contact_monitor` at 50 Hz from `on_query_contact()`. (Updated 2026-10-30, Swapnil's ruling: removed with `ContactMonitorState`. The diagnostics gain `last_trip_thresholds_nm`, `last_trip_q_rad` and `last_trip_qd_rad_s`. A read-only parameter `contact_debug_topic` (default `false`) publishes `sensor_msgs/JointState` on `/contact_monitor/residual` at 50 Hz, the seven arm joint names and `effort` = `r`, reliable QoS; not created when false or with detection `off`. `ContactServer` takes `bool debug_topic` and the arm joint names after `detection`.)
  - Diagnostics task `"Contact detection"` with the same hardware ID `kinova_gen3`:
    - OK: `armed (profile P)`
    - WARN: `settling`, `observe mode: not stopping the arm; N would-be trips[, last on joint J]`, `DISABLED (contact_detection=off)`
    - ERROR: `stopped on contact: joint N, |r| = x N·m > θ = y N·m at the trip (profile now P); call /clear_faults`
  - **(Plan call:** a separate task next to `Arm link`, in the same updater style. That way `LinkServer` doesn't need to know the Supervisor.)
- `LinkServer` takes `std::function<bool()> clear_contact` (empty when detection is off). `on_clear` calls it first. If the link is faulted, it continues the existing path. If not faulted and a contact was cleared, it returns `success=true`, message `"contact stop cleared"`. Otherwise it returns the existing `"not faulted (<state>)"`.
- `bringup_node.cpp`:
  - Build the `ContactMonitor` and `FloatMode` on `*dyn` (the RT-thread `Dynamics`, already shared by the modes on that thread), and call `exec.attach_contact(...)` unless detection is `off`.
  - Set `deps.contact` and `deps.contact_profiles`.
  - Log `contact detection: <mode>, default θ = [...] N·m, ceiling [...]` at startup.
  - With `off` or `observe`, also log a WARN.
  - On invalid config, FATAL and exit non-zero.

- [x] **Step 1: Write failing tests:**

```cpp
TEST(ContactParams, DefaultsMatchSpec)                     // values from Global Constraints
TEST(ContactParams, InvalidThrowsNamingParameter)          // default above ceiling -> message contains "contact_thresholds_default" and "joint"
TEST(ContactServer, RefusesBadRequests)                    // Review Focus 5: custom with 6 values, NaN, above ceiling, zero token -> accepted=false, message names the problem
TEST(ContactServer, AppliesNamedProfile)
TEST(ContactServer, PublishesState)                        // (Updated 2026-10-30: replaced by DebugTopicOffByDefault, DebugTopicNotCreatedWithDetectionOff, DebugTopicPublishesResidual, ContactDiagnostics.TripRecordKeyValues)
TEST(ContactServer, DiagnosticsEachPhase)                  // settling/armed/latched/observe/off -> level + message
TEST(LinkServer, ClearFaultsClearsContactWhenRunning)      // link kRunning + latched -> success, "contact stop cleared"
TEST(LinkServer, ClearFaultsClearsBoth)                    // faulted + latched -> contact cleared and link clear requested
```

Use a fake `CommandSink`, adding `fake_contact_sink.h` next to the other fakes.

- [x] **Step 2: Run** `colcon test ... -R "contact_|link_server_test"`. Expected: FAIL.
- [x] **Step 3: Implement.** Write the docs.
- [x] **Step 4: Run** the full `colcon test --packages-select kinova_gen3_ros2`, then `make smoke`. Expected: PASS, and the smoke log shows the `contact detection: enforce` line.
- [x] **Step 5: Commit** the explicit paths, with message `feat: contact detection surface (service, topic, diagnostics, /clear_faults)`. Open a PR (body through `unslop`) linking the driver and interfaces PRs, plus #24.

## Part D: on-arm verification (no code; each motion batch confirmed by Swapnil)

Status 2026-10-30: the still floor (1), the tours in both modes (2, with the default
doubled after them) and the gripper force to trip at home (part of 3) are in the
driver's `docs/integration/contact_detection_bench.md` and this repo's
`docs/interface.md` (False positives). The rest is open.

Ask Swapnil before taking abra or the arm. Use the driver branch image through `make real CORE_REF=...`, swapping sheppy-arm as in memory `dojo-native-node-testing`. Before each batch, state what will move, how fast, and in position or impedance mode, and wait for "go".

1. `observe`, no motion, 60 s: still floor per joint.
2. `observe`, the teammate's `tour.py` at speed 0.6, then 1.0, nobody touching: motion RMS and peak per joint, plus `would_trips`. Set `default` to ≈ 3× the peak, then rerun and expect `would_trips == 0`.
3. `observe`, still arm, slow hand pushes by Swapnil on the forearm and the tool: record `r` against `/ee_state` wrench, which is exact at rest. This maps θ to force.
4. `enforce`, slow move, hand push: trip, float, latch, `-11` on the goal, `/clear_faults`, hold. Then at demo speed.
5. Force build-up after a trip; the position-mode → float window (look for the #92 droop); the RT cycle time with and without the monitor (`--rt-csv`).
6. `observe` with real payloads (spoon, container): `r` while carrying.
7. Pose check: from logged `q` along the feeding approach, the smallest gripper or forearm force that crosses a θ, in the worst direction. Script in `tools/contact/`.

Write the results as a bench report in the driver's `docs/integration/`, linked from #24. Replay recorded CSVs through `contact_replay` to tune offline between sessions.
