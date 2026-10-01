"""Checks for speed_scale on ExecuteJointTrajectory.

Scoped deliberately to what the NODE owns. The other new field —
orientation_hold — only means anything once cuRobo is planning, and the whole
conformance suite is currently driver-facing with no planner dependency. Adding
GoToEEPose here would make the suite unrunnable without a GPU stack, so the
constraint cases live in test/sweep_constraints.py instead, which expects a
planner and tables the results.

The refusal checks command nothing: a rejected goal never reaches the arm, so
they carry needs_motion=False and are safe to run against a live cell. Only the
dilation check moves, and it moves one wrist joint a fraction of a radian.

The bound worth defending is that out-of-range is REFUSED rather than CLAMPED.
Clamping a below-floor scale up to the floor would run the arm FASTER than the
caller asked for, which is the wrong direction for a mistake to resolve in.
"""

import time

from builtin_interfaces.msg import Duration as DurationMsg
from rammp_arm_interfaces.action import ExecuteJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from harness import FAIL, PASS, REGISTRY, Result, tok

SEC = "speed"

SUCCESSFUL = 0
JOINT = 6  # the wrist, matching checks_motion: lightest link, least able to harm
DELTA = 0.20  # rad
BASE_DURATION = 3.0  # s at scale 1.0; the half-speed leg takes ~6 s

# The driver's floor, from kinova::interface::kMinSpeedScale. Anything below it
# is refused; zero would stop the trajectory clock and hang the goal forever.
MIN_SPEED_SCALE = 0.01


def _goal(ctx, session_token, speed_scale, duration=BASE_DURATION):
    q = ctx.joint_positions()
    if q is None:
        return None
    target = list(q)
    target[JOINT] += DELTA
    g = ExecuteJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    for frac in (0.0, 1.0):
        p = JointTrajectoryPoint()
        p.positions = q if frac == 0.0 else target
        p.time_from_start = DurationMsg(
            sec=int(frac * duration), nanosec=int((frac * duration % 1) * 1e9)
        )
        g.trajectory.points.append(p)
    g.control_mode = 0  # POSITION
    g.preemption = 1  # LATEST_WINS
    g.sender_id = "conformance"
    g.token = tok(session_token)
    g.speed_scale = speed_scale
    return g


def _send(ctx, goal, timeout=40.0):
    """Send a goal; return (accepted, result_or_None, wall_seconds_to_settle)."""
    cli = ActionClient(ctx.n, ExecuteJointTrajectory, "execute_joint_trajectory")
    if not cli.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("execute_joint_trajectory server never appeared")
    t0 = time.time()
    fut = cli.send_goal_async(goal)
    end = time.time() + timeout
    while time.time() < end and not fut.done():
        ctx.spin(0.02)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return False, None, time.time() - t0
    rf = gh.get_result_async()
    while time.time() < end and not rf.done():
        ctx.spin(0.02)
    if not rf.done():
        return True, None, time.time() - t0
    return True, rf.result().result, time.time() - t0


def _refusal_check(ctx, scale, label):
    """Shared body: a goal carrying `scale` must be refused outright."""
    session_tok = ctx.acquire("conformance-speed").token
    g = _goal(ctx, session_tok, scale)
    if g is None:
        return Result("", FAIL, "no /joint_states, cannot build a goal")
    accepted, result, _ = _send(ctx, g)
    if accepted:
        # It moved, or is moving. Say so precisely -- a clamp is the specific
        # failure this check exists to catch.
        code = result.error_code if result else "never settled"
        return Result(
            "",
            FAIL,
            f"{label} was ACCEPTED (result={code}); it must be refused, not "
            f"clamped -- a clamp runs the arm faster than the caller asked",
        )
    return Result("", PASS, f"{label} refused before execution")


@REGISTRY.add(SEC, "a speed_scale below the floor is refused, not clamped")
def check_below_floor_refused(ctx):
    return _refusal_check(ctx, MIN_SPEED_SCALE / 5.0, "speed_scale=0.002")


@REGISTRY.add(SEC, "a speed_scale above 1.0 is refused")
def check_above_one_refused(ctx):
    return _refusal_check(ctx, 1.5, "speed_scale=1.5")


@REGISTRY.add(SEC, "a zero speed_scale is refused (it would stop the clock)")
def check_zero_refused(ctx):
    return _refusal_check(ctx, 0.0, "speed_scale=0.0")


@REGISTRY.add(SEC, "a NaN speed_scale is refused")
def check_nan_refused(ctx):
    return _refusal_check(ctx, float("nan"), "speed_scale=NaN")


@REGISTRY.add(
    SEC, "halving speed_scale roughly doubles the wall time", needs_motion=True
)
def check_dilation_stretches_wall_time(ctx):
    """The same trajectory at 1.0 and at 0.5. Same path, same endpoints; only the
    clock differs, so the second should take about twice as long.

    The arm ends where it started: the full-speed leg moves +DELTA and the
    half-speed leg is built fresh from the measured q, so it moves +DELTA again.
    That is intentional -- each leg is timed from its own start, and neither
    depends on the other's endpoint.
    """
    session_tok = ctx.acquire("conformance-speed").token

    g_full = _goal(ctx, session_tok, 1.0)
    if g_full is None:
        return Result("", FAIL, "no /joint_states, cannot build a goal")
    accepted, res_full, wall_full = _send(ctx, g_full)
    if not accepted or res_full is None or res_full.error_code != SUCCESSFUL:
        code = res_full.error_code if res_full else "rejected/never settled"
        return Result("", FAIL, f"the full-speed leg did not succeed ({code})")

    g_half = _goal(ctx, session_tok, 0.5)
    if g_half is None:
        return Result("", FAIL, "no /joint_states for the second leg")
    accepted, res_half, wall_half = _send(ctx, g_half)
    if not accepted or res_half is None or res_half.error_code != SUCCESSFUL:
        code = res_half.error_code if res_half else "rejected/never settled"
        return Result("", FAIL, f"the half-speed leg did not succeed ({code})")

    if wall_full <= 0.1:
        return Result("", FAIL, f"the full-speed leg took {wall_full:.2f}s; too short to compare")
    ratio = wall_half / wall_full
    detail = f"{wall_full:.2f}s at 1.0 vs {wall_half:.2f}s at 0.5 (ratio {ratio:.2f})"
    # A wide band on purpose: both legs carry the same fixed action round-trip and
    # settle overhead, which pulls the ratio below 2.0. The failure this catches is
    # a scale that never reached the executor at all, which lands near 1.0.
    if not 1.5 <= ratio <= 2.5:
        return Result("", FAIL, f"expected ~2x, got {detail}")
    return Result("", PASS, detail)
