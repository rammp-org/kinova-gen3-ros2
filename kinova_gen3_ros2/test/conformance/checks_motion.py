"""Checks that command the arm.

Motions are deliberately small and slow -- one joint, a fraction of a radian, over
seconds -- because the point is to observe the CONTROL SURFACE, not the trajectory. An
operator should be on the e-stop for this section.

Two of these have never been exercised end to end and are the reason this file exists:

  * an e-stop must terminate an in-flight goal with HALTED (-9). Core settles it in the
    sampler; nothing had ever confirmed it reaches a ROS action client.
  * under kEnforced a cancel must actually cancel. A cancel carries no payload in the
    ROS action protocol, so the driver replays the goal's stored token; get that wrong
    and the arm keeps moving while the client believes it stopped.
"""
import time

from builtin_interfaces.msg import Duration as DurationMsg
from kinova_gen3_interfaces.action import ExecuteJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from harness import FAIL, PASS, REGISTRY, ZERO_TOKEN, Result, tok

SEC = "motion"

HALTED, SUCCESSFUL = -9, 0
JOINT = 6            # the wrist: lightest link, least able to do harm if a sign is wrong
DELTA = 0.25         # rad
DURATION = 6.0       # s -- slow enough to e-stop in the middle of it


def _goal(ctx, session_token, delta=DELTA, duration=DURATION):
    q = ctx.joint_positions()
    if q is None:
        return None
    target = list(q)
    target[JOINT] += delta
    g = ExecuteJointTrajectory.Goal()
    g.trajectory = JointTrajectory()
    for frac in (0.0, 1.0):
        p = JointTrajectoryPoint()
        p.positions = q if frac == 0.0 else target
        p.time_from_start = DurationMsg(sec=int(frac * duration),
                                        nanosec=int((frac * duration % 1) * 1e9))
        g.trajectory.points.append(p)
    g.control_mode = 0                       # POSITION
    g.preemption = 1                         # LATEST_WINS
    g.sender_id = "conformance"
    g.token = tok(session_token)
    return g


def _send(ctx, goal, wait_result=True, timeout=25.0):
    """Send a goal. Returns (goal_handle, result_or_None)."""
    cli = ActionClient(ctx.n, ExecuteJointTrajectory, "execute_joint_trajectory")
    if not cli.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("execute_joint_trajectory server never appeared")
    fut = cli.send_goal_async(goal)
    end = time.time() + timeout
    while time.time() < end and not fut.done():
        ctx.spin(0.02)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return gh, None
    if not wait_result:
        return gh, None
    rf = gh.get_result_async()
    while time.time() < end and not rf.done():
        ctx.spin(0.02)
    return gh, (rf.result().result if rf.done() else None)


@REGISTRY.add(SEC, "an e-stop terminates an IN-FLIGHT goal with HALTED (-9)",
              needs_motion=True)
def check_estop_halts_goal(ctx):
    ctx.estop(False, reason="conformance: arming for the halt test")
    session_tok = ctx.acquire("conformance-motion").token
    g = _goal(ctx, session_tok)
    if g is None:
        return Result("", FAIL, "no /joint_states, cannot build a goal")
    gh, _ = _send(ctx, g, wait_result=False)
    if gh is None or not gh.accepted:
        return Result("", FAIL, "the goal was not accepted; cannot test the halt")

    ctx.spin(1.5)                                   # let it get moving
    ctx.estop(True, reason="conformance: halting an in-flight goal", settle=0.5)

    rf = gh.get_result_async()
    end = time.time() + 15.0
    while time.time() < end and not rf.done():
        ctx.spin(0.02)
    if not rf.done():
        return Result("", FAIL, "the goal never settled after an e-stop -- the client "
                                "hangs forever, which is the orphaned-goal failure")
    code = rf.result().result.error_code
    ctx.estop(False, reason="conformance cleanup")
    if code != HALTED:
        return Result("", FAIL, f"settled with {code}, expected {HALTED} HALTED")
    return Result("", PASS, f"goal settled HALTED ({code}) after the e-stop")


@REGISTRY.add(SEC, "enforced: a goal with NO token is refused", needs_motion=True,
              needs_mode="enforced")
def check_enforced_rejects_untokened(ctx):
    ctx.revoke("conformance: ensuring no owner")
    g = _goal(ctx, ZERO_TOKEN)
    gh, _ = _send(ctx, g, wait_result=False)
    if gh is not None and gh.accepted:
        gh.cancel_goal_async()
        ctx.spin(1.0)
        return Result("", FAIL, "an untokened goal was ACCEPTED under kEnforced")
    return Result("", PASS, "refused, as kEnforced requires")


@REGISTRY.add(SEC, "enforced: a goal WITH the token runs to completion",
              needs_motion=True, needs_mode="enforced")
def check_enforced_accepts_tokened(ctx):
    session_tok = ctx.acquire("conformance-motion").token
    g = _goal(ctx, session_tok, delta=0.15, duration=4.0)
    gh, res = _send(ctx, g)
    if gh is None or not gh.accepted:
        return Result("", FAIL, "a correctly tokened goal was refused")
    if res is None:
        return Result("", FAIL, "the goal never produced a result")
    if res.error_code != SUCCESSFUL:
        return Result("", FAIL, f"settled {res.error_code}, expected 0 SUCCESSFUL")
    return Result("", PASS, "accepted and completed")


@REGISTRY.add(SEC, "enforced: CANCEL actually cancels (the stored-token replay)",
              needs_motion=True, needs_mode="enforced")
def check_enforced_cancel(ctx):
    session_tok = ctx.acquire("conformance-motion").token
    g = _goal(ctx, session_tok)
    gh, _ = _send(ctx, g, wait_result=False)
    if gh is None or not gh.accepted:
        return Result("", FAIL, "the goal was not accepted; cannot test cancel")
    ctx.spin(1.5)
    gh.cancel_goal_async()
    rf = gh.get_result_async()
    end = time.time() + 15.0
    while time.time() < end and not rf.done():
        ctx.spin(0.02)
    if not rf.done():
        return Result("", FAIL, "the goal never settled after cancel. This is the "
                                "silent-cancel bug: a ROS cancel carries no token, so "
                                "the Arbiter refuses it and the arm keeps moving.")
    code = rf.result().result.error_code
    if code == SUCCESSFUL:
        return Result("", FAIL, "the goal ran to COMPLETION despite being cancelled -- "
                                "the cancel never reached the supervisor")
    return Result("", PASS, f"cancel honoured, settled {code}")
