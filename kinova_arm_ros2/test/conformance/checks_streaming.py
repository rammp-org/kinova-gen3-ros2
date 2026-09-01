"""The streaming tier.

Every check here that streams setpoints streams the arm's OWN MEASURED configuration
back at it. That is not timidity: it proves the setpoint path, the admission check and
the deadline refresh while commanding the arm exactly where it already is, so the
plumbing is under test and the arm is not.
"""
import time

from kinova_arm_interfaces.msg import JointSetpoint, PoseSetpoint
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from harness import FAIL, PASS, REGISTRY, Result, tok

SEC = "streaming"

# Best-effort, depth 1 -- core's setpoints are absolute and latest-wins, so this is the
# QoS the node advertises. A reliable publisher would not match it.
SETPOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)


def _hold(ctx, session_token, seconds, topic="/setpoint/joint_position"):
    """Stream the measured configuration back at the arm for `seconds`.

    Commands the arm where it already is, so the session stays alive with no motion.
    Returns the number of setpoints sent, or None if there was no joint state to echo.
    """
    q = ctx.joint_positions()
    if q is None:
        return None
    pub = ctx.n.create_publisher(JointSetpoint, topic, SETPOINT_QOS)
    end = time.time() + seconds
    sent = 0
    while time.time() < end:
        m = JointSetpoint()
        m.values = [float(v) for v in q]
        m.token = tok(session_token)
        pub.publish(m)
        sent += 1
        ctx.spin(0.02)
    return sent


@REGISTRY.add(SEC, "/list_controllers reports the registry with live availability")
def check_list_controllers(ctx):
    rows = ctx.list_controllers()
    if not rows:
        return Result("", FAIL, "no controllers listed")
    by = {r.name: r for r in rows}
    for want in ("joint_position", "joint_impedance", "ee_pose_impedance",
                 "ee_pose_position", "joint_torque", "joint_velocity", "ee_twist",
                 "cartesian_impedance"):
        if want not in by:
            return Result("", FAIL, f"controller '{want}' is missing from the registry")
    if by["cartesian_impedance"].available:
        return Result("", FAIL, "cartesian_impedance reports available, but core has no "
                                "kEeWrench and it needs multi-channel sessions")
    for r in rows:
        if r.available and len(r.channels) != 1:
            return Result("", FAIL, f"'{r.name}' is available with {len(r.channels)} "
                                    "channels; core admits one SetpointKind per session")
    avail = sorted(n for n, r in by.items() if r.available)
    return Result("", PASS, f"{len(avail)}/{len(rows)} available: {avail}")


@REGISTRY.add(SEC, "opening an UNAVAILABLE controller is refused locally")
def check_open_unavailable(ctx):
    r = ctx.open_stream("cartesian_impedance", timeout_s=0.5)
    if r.accepted:
        ctx.close_stream()
        return Result("", FAIL, "cartesian_impedance opened; core cannot back it")
    if r.error_code != -10:
        return Result("", FAIL, f"error_code {r.error_code}, expected -10 STREAM_REJECTED")
    return Result("", PASS, f"refused: {r.message}")


@REGISTRY.add(SEC, "an UNKNOWN controller name is refused without reaching core")
def check_open_unknown(ctx):
    r = ctx.open_stream("no_such_controller", timeout_s=0.5)
    if r.accepted:
        ctx.close_stream()
        return Result("", FAIL, "an unknown controller name was accepted")
    return Result("", PASS, f"refused: {r.message}")


@REGISTRY.add(SEC, "timeout_s <= 0 is refused: an unbounded stream has no safe-stop")
def check_open_no_deadline(ctx):
    r = ctx.open_stream("joint_position", timeout_s=0.0)
    if r.accepted:
        ctx.close_stream()
        return Result("", FAIL, "a session opened with no deadline")
    return Result("", PASS, f"refused: {r.message}")


@REGISTRY.add(SEC, "open returns the channel, and /stream_status agrees")
def check_open_close(ctx):
    session_token = ctx.acquire("conformance-stream").token
    r = ctx.open_stream("joint_impedance", timeout_s=2.0, token=session_token)
    if not r.accepted:
        return Result("", FAIL, f"open refused: {r.message}")
    if list(r.channels) != ["joint_position"]:
        return Result("", FAIL, f"channels={list(r.channels)}, expected ['joint_position']")
    ss = ctx.stream_status()
    if not ss.open or ss.controller != "joint_impedance":
        return Result("", FAIL, f"status says open={ss.open} controller='{ss.controller}'")
    ctx.close_stream(session_token)
    ss = ctx.stream_status()
    if ss.open:
        return Result("", FAIL, "still open after close_stream")
    if ss.controller:
        return Result("", FAIL, f"closed but still labelled '{ss.controller}'")
    return Result("", PASS, "opened on 'joint_position', closed cleanly")


@REGISTRY.add(SEC, "setpoints refresh the deadline (session outlives timeout_s)")
def check_deadline_refresh(ctx):
    session_token = ctx.acquire("conformance-stream").token
    r = ctx.open_stream("joint_position", timeout_s=0.5, token=session_token)
    if not r.accepted:
        return Result("", FAIL, f"open refused: {r.message}")
    sent = _hold(ctx, session_token, seconds=3.0)          # 6x the deadline
    if sent is None:
        return Result("", FAIL, "no /joint_states, cannot build a hold setpoint")
    ss = ctx.stream_status()
    if not ss.open:
        return Result("", FAIL, f"session closed while streaming {sent} setpoints at "
                                "~50 Hz into a 0.5 s deadline -- they are not landing")
    ctx.close_stream(session_token)
    return Result("", PASS, f"{sent} setpoints held the session open past 6x timeout_s")


@REGISTRY.add(SEC, "the session expires once setpoints stop")
def check_deadline_expiry(ctx):
    session_token = ctx.acquire("conformance-stream").token
    r = ctx.open_stream("joint_position", timeout_s=0.5, token=session_token)
    if not r.accepted:
        return Result("", FAIL, f"open refused: {r.message}")
    _hold(ctx, session_token, seconds=1.0)
    ctx.spin(2.5)                                          # 5x the deadline, silent
    ss = ctx.stream_status()
    if ss.open:
        ctx.close_stream(session_token)
        return Result("", FAIL, "session still open 2.5 s after the last setpoint; the "
                                "safe-stop deadline is not firing")
    return Result("", PASS, "session expired and /stream_status reported it")


@REGISTRY.add(SEC, "a setpoint on the WRONG channel is counted, not applied")
def check_wrong_channel(ctx):
    session_token = ctx.acquire("conformance-stream").token
    r = ctx.open_stream("joint_position", timeout_s=3.0, token=session_token)
    if not r.accepted:
        return Result("", FAIL, f"open refused: {r.message}")
    before = ctx.stream_status().rejected_count
    pub = ctx.n.create_publisher(PoseSetpoint, "/setpoint/pose", SETPOINT_QOS)
    for _ in range(40):
        m = PoseSetpoint()
        m.pose.orientation.w = 1.0
        m.token = tok(session_token)
        pub.publish(m)
        ctx.spin(0.02)
    ss = ctx.stream_status()
    after, still_open = ss.rejected_count, ss.open
    ctx.close_stream(session_token)
    if after <= before:
        return Result("", FAIL, f"rejected_count did not move ({before} -> {after}); "
                                "wrong-kind setpoints are meant to be counted")
    if not still_open:
        return Result("", FAIL, "the session died from wrong-channel traffic; a rejected "
                                "setpoint must not refresh the deadline OR kill it")
    return Result("", PASS, f"rejected_count {before} -> {after}, session survived")
