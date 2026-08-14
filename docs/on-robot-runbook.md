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
