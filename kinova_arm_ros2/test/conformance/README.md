# Conformance runner

Exercises every ROS control surface against a **running** `kinova_arm_node` — the
inputs and the outputs — and asserts the behaviour, rather than printing values for a
human to eyeball.

```bash
# 1. start the node yourself, with the mode you want to test
ros2 run kinova_arm_ros2 kinova_arm_node --sim --urdf models/gen3_7dof_2f85.urdf \
  --ros-args -p arbitration_mode:=enforced

# 2. attach
python3 run_conformance.py                 # everything
python3 run_conformance.py --no-motion     # nothing that commands the arm
python3 run_conformance.py --sections streaming
python3 run_conformance.py --list
```

It **never launches a node**. `arbitration_mode` is read-only at launch, so the runner
reads the mode from `/control_status` and **skips** the checks that need the other one,
naming them — a skip is reported, never silently counted as a pass.

Exit code is non-zero if anything failed.

## On real hardware

**Be on the e-stop.** The `motion` section commands the arm: one joint, ≤0.25 rad, over
several seconds. Claim the cell first (see the `dojo` skill) so nothing else is driving.

Ordering is deliberate — `state`, `arbitration`, `streaming`, `motion` — so nothing
commands the arm until the stop path has been proven **in this session**. If the e-stop
checks fail the run aborts before any motion.

The streaming checks stream the arm's **own measured configuration** back at it. That
exercises the setpoint path, the admission check and the deadline refresh while
commanding the arm exactly where it already is.

The runner restores state on exit whatever happens: closes any session, releases
ownership, clears the e-stop.

## What it covers

| section | |
|---|---|
| `state` | `/joint_states` names, `/ee_state` plausibility, both from one pump tick, `/control_status` and `/stream_status` latched and self-consistent, `/diagnostics` carrying both REP 107 tasks |
| `arbitration` | e-stop engage/clear, the staleness **asymmetry** (stale clear refused, stale engage honoured, unstamped accepted), acquire, seizure bumping `generation`, release with the wrong token refused, operator revoke |
| `streaming` | `/list_controllers` availability, unavailable and unknown controllers refused, `timeout_s<=0` refused, open/close, setpoints refreshing the deadline, expiry once they stop, wrong-channel setpoints counted but not applied |
| `motion` | **an e-stop settling an in-flight goal `-9`**, and under `enforced`: an untokened goal refused, a tokened one completing, and **cancel actually cancelling** |
| `gripper` | `/gripper_state` plausible, the actuated joint in `/joint_states` within its URDF limits and **no mimics published**, an untokened setpoint counted as rejected, a tokened one moving the gripper |

Those last two had never been exercised end to end. The cancel one guards a specific
trap: a ROS action cancel carries no payload, so the driver replays the goal's stored
token — get it wrong and the arm keeps moving while the client believes it stopped.

## Two gotchas worth knowing

**Tokens come back as numpy.** rclpy hands `uint8[16]` back as a numpy array, so
`list(token)` yields `numpy.uint8` and the outgoing message assertion rejects it. Use
`harness.tok()`.

**Latched topics need a re-subscribe, not a cache clear.** `/control_status` and
`/stream_status` are `transient_local` and publish *on change*. Clearing a cached value
and waiting hangs forever when nothing changes; the latched sample is delivered on
match. `Ctx.latest(..., fresh=True)` recreates the subscription for those topics.
