# On-Robot Runbook — ExecuteJointTrajectory (attended)

**Attended only.** Never connect to the arm unattended. This mirrors the core
driver's Plan-1 on-robot step: small / slow / distal, e-stop in hand.

Prereqs: arm powered, homed to a safe pose, workspace clear, e-stop within reach,
arm IP reachable (`ping 192.168.1.10`). Build + run happen on abra (aarch64).

## 1. Build the KORTEX-enabled workspace (explicit flag)

The CMake cache persists `KINOVA_ENABLE_KORTEX` across colcon rebuilds, so ALWAYS
pass the flag explicitly — never rely on a bare `colcon build` to be in the mode
you expect.

```sh
# from muk:
bash scripts/abra_colcon.sh --cmake-args \
  -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64
```

Confirm the node linked KORTEX (not the sim binary): the `kinova_arm_node` binary
is ~9.7MB (vs ~1.5MB sim) and `strings` on it shows `KortexTransport::connect`.

**When done on the arm, rebuild sim-only explicitly** so the installed workspace
isn't left KORTEX-linked:
`bash scripts/abra_colcon.sh --cmake-args -DKINOVA_ENABLE_KORTEX=OFF`

## 2. Dry-run / read-only (no motion) — and confirm measured pose

Launch the node against the arm; command nothing. Confirm it connects, advertises
the action, and — critically — that `/joint_states` reports the arm's REAL current
pose (the client seeds the trajectory from this, so it must be correct).

```sh
# on abra (terminal A):
source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash
cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver
ros2 run kinova_arm_ros2 kinova_arm_node --ip 192.168.1.10 --urdf models/gen3_7dof_2f85.urdf
# expect: "kinova_arm_node up (real); action: /execute_joint_trajectory"

# on abra (terminal B) — note: /joint_states is best-effort QoS:
ros2 action list                                             # expect /execute_joint_trajectory
ros2 topic echo --once --qos-reliability best_effort /joint_states   # expect the arm's REAL joint angles
```

Sanity-check the echoed `position[]` against the arm's actual pose (Kinova web app
or eyeball). If it reads all-zeros while the arm is clearly not at zero, STOP —
the seed would be wrong. Ctrl-C terminal A to stop (clean shutdown). Nothing
commanded.

## 3. First move — small/slow single joint (attended, e-stop in hand)

The client now **seeds the trajectory from the live measured pose** (waypoint-0 =
current `/joint_states`), then moves ONLY `--joint` by `--delta` — so on a real
arm at any pose it commands a small LOCAL move, never a jump to zero. Send a
conservative goal from terminal B: joint 6 (most distal), 0.10 rad (~5.7°), 1.2 s,
position mode, live path tolerance 0.2:

```sh
python3 /tmp/kinova-ros2-ws/src/kinova_arm_ros2/kinova_arm_ros2/test/send_trajectory.py \
  --joint 6 --mode position --delta 0.10 --dur 1.2 --path-tol 0.2 --expect 0
```

The client prints the measured start (`measured start: joint6=<x> -> target <x+0.10>`)
and, each feedback tick, the MEASURED joint angle (`measured_j6=…`). Watch:
- the printed measured start matches the arm's real pose,
- `measured_j6` advances from start toward start+0.10 as the arm moves,
- the result settles `error_code=0` / `STATUS_SUCCEEDED`.

Because the feedback's `actual` field is the driver's live measured q, the
`measured_j6` stream IS the confirmation of real motion (no separate query needed).
Cross-check the final `measured_j6` ≈ start+0.10.

## 4. Record the run

Append below: date, exact command, observed motion (measured joint delta),
result code, and any `faults`/`dropped`/`majflt` reported by the node's telemetry
drain. Stop on anything unexpected — e-stop, then investigate.

### Runs
- (append results here)
- **2026-08-12 (attended, e-stop):** First run surfaced a core bug — a well-tracking
  j6 +0.10 move false-aborted (`PATH_TOLERANCE_VIOLATED`) at ~71% because the
  supervisor sampler injected q=0 into the divergence guard on a rare failed
  feedback-snapshot read. Fixed in the core (reuse last-good q; commit `dd3e57e`).
  After the fix, re-verified on the arm:
  - j6 +0.40 rad over 2.0s → SUCCEEDED, rest on target (1.6423→2.0423).
  - coordinated two-joint (j5 +0.40, j6 −0.40) over 2.5s → SUCCEEDED, both on target
    (j5 0.960→1.360, j6 2.042→1.642). Multi-joint client: `--joint 5,6 --delta 0.4,-0.4`
    (note: put a positive delta first — argparse treats a leading `-` value as a flag).
- **2026-09-01 (attended, e-stop):** First **containerized** real-arm run, and the first time
  the conformance suite pointed at hardware rather than sim. Image built with
  `make build-real CORE_REF=integration/velocity-and-stream-status` (core `6c741d0`) — the
  container path was chosen deliberately over `scripts/abra_colcon.sh`, because the local core
  working tree sits on the unpushed `feat/gripper-tier-spec`, which is ahead of anything the
  ROS surface can build against. Node run with `arbitration_mode:=enforced`.
  **28/28 passed, 0 failed, 0 skipped.** No faults, dropped samples, or major page faults in the
  telemetry drain. `/ee_state` read p=(0.457, 0.001, 0.434), |p|=0.630 m at home.
  - First attempt aborted before any conformance check completed and the node exited. **The
    cause was not captured** — that container ran with `--rm`, so its logs died with it. Run
    the node WITHOUT `--rm` on hardware; a lost log costs a whole attended session.
    Confirmed: the arm was parked folded up with **joint 4 at -2.660018 rad, 1.8e-5 rad outside
    its [-2.66, 2.66] URDF limit**, and the operator independently observed the arm against a
    joint limit. Suspected, not proven: that pose breaks a premise the streaming section rests
    on — it streams the arm's own *measured* configuration back at it, assuming that commanding
    the arm where it already is must be admissible, which stops being true when the measured
    pose sits microradians past a limit. Re-run from home (joint 4 at -2.269, 0.39 rad of
    margin) passed clean. To settle it, reproduce at the limit with logs kept. Worth hardening
    either way: the runner could clamp the echoed setpoint into limits, or the driver could
    admit an epsilon-outside-limit setpoint that equals the measured pose.
- **2026-09-01 (attended, e-stop) — velocity-mode teleop, two findings.** Hand-flying the EE
  streaming controllers with an Xbox pad (`test/teleop_xbox.py`, throwaway) surfaced a real
  hardware behaviour and a real driver crash. Same image as the conformance run above
  (core `6c741d0`), node in `arbitration_mode:=enforced`.

  **1. A zero velocity command does not hold joint 2 — the arm creeps.** An `ee_twist`
  session streaming zero twist, hands off the sticks, drifted the tool ~22 cm and rotated
  the shoulder +1.475 rad before it was stopped. Isolated afterwards with
  `test/zero_setpoint_probe.py`, which streams a hard zero on each controller in turn:

  | controller | DLS solve | posture term | per-joint drift, 6 s |
  |---|---|---|---|
  | `joint_velocity` | no (passthrough) | no | `+0.000 +0.227 +0.000 -0.000 +0.000 -0.000 +0.000` |
  | `ee_twist` | yes | yes | `+0.000 +0.234 +0.000 +0.000 +0.000 +0.000 -0.000` |

  Only joint 2 moves, ~0.038 rad/s, in the gravity direction; the other six hold at exactly
  0.000. `joint_velocity` is the documented passthrough — no solve, no null-space posture
  bias — and it drifts identically, which rules out the solver and the posture term as the
  cause. At 0.038 rad/s the incident's +1.475 rad is ~39 s of creep, matching the session
  length. Conclusion: the actuator's own velocity servo is not rejecting the shoulder's
  gravity load at a zero command. Not a software fault in this driver. Raised with Kinova.

  Consequence for clients: **velocity mode is not a hold.** Zero velocity means "stop
  driving", not "stay put". Do not park an arm in `joint_velocity` or `ee_twist` and stream
  zeros. `ee_pose_position` holds; use it for hand-flying.

  Note the guide's "**Stiff by contract.** It does not yield to contact and makes no attempt
  to" reads as a hold guarantee and should be qualified — the arm does not hold its own
  weight at the shoulder.

  **2. An external servoing-mode change kills the node.** Jogging the arm from the Kinova
  web app while the driver was connected took it out of low-level servoing; the driver's
  next write threw and nothing caught it:

      terminate called after throwing an instance of 'Kinova::Api::KDetailedException'
        what():  Device error, Error sub type=WRONG_SERVOING_MODE
        description: Wrong servoing mode, must be low level servoing mode

  Container exited 133 (`std::terminate`). Crash was 6m41s AFTER the arm had already
  stopped, so it is independent of finding 1 — confirmed from `docker inspect` timestamps
  against the teleop log. A KORTEX device error should be handled and surfaced on
  `/diagnostics`, not abort the process.

  Process note: the first container was run with `--rm`, which destroyed the logs of an
  earlier exit and cost a diagnosis. The crash above was only readable because the second
  run kept them. Never use `--rm` on hardware.
