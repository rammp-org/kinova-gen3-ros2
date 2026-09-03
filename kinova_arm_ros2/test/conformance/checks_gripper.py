"""The gripper tier.

Ordered AFTER motion for the same reason motion is last: these command hardware. The
gripper commands are small and slow, and open/close on empty air is safe -- but it is
still actuation, so the e-stop must already be proven in this session.
"""
import time

from kinova_arm_interfaces.msg import GripperSetpoint, GripperState
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from harness import FAIL, PASS, REGISTRY, SKIP, ZERO_TOKEN, Result, tok

SEC = "gripper"
SP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
KNUCKLE = "robotiq_85_left_knuckle_joint"


@REGISTRY.add(SEC, "/gripper_state publishes and is self-consistent")
def check_gripper_state(ctx):
    g = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if g is None:
        return Result("", FAIL, "no /gripper_state")
    if not (0.0 <= g.position <= 1.0):
        return Result("", FAIL, f"position {g.position} outside [0,1]")
    if not (0.0 <= g.effort <= 1.0):
        return Result("", FAIL, f"effort {g.effort} outside [0,1] -- it is a fraction")
    return Result("", PASS, f"present={g.present} position={g.position:.3f} "
                            f"effort={g.effort:.3f} current={g.current:.3f}A")


@REGISTRY.add(SEC, "the actuated joint appears in /joint_states, within its URDF limits")
def check_knuckle_in_joint_states(ctx):
    js = ctx.latest("joint_states", timeout=6.0, fresh=True)
    if js is None or KNUCKLE not in js.name:
        return Result("", FAIL, f"{KNUCKLE} missing from /joint_states")
    i = list(js.name).index(KNUCKLE)
    q = js.position[i]
    if not (0.0 <= q <= 0.8):
        return Result("", FAIL, f"{KNUCKLE}={q} outside the URDF limit [0, 0.8]")
    # Only the actuated joint: RSP derives the five mimics.
    mimics = [n for n in js.name if "robotiq" in n and n != KNUCKLE]
    if mimics:
        return Result("", FAIL, f"mimic joints published too: {mimics}")
    return Result("", PASS, f"{KNUCKLE}={q:.4f} rad, no mimics published")


@REGISTRY.add(SEC, "enforced: an untokened gripper setpoint is refused",
              needs_motion=True, needs_mode="enforced")
def check_untokened_refused(ctx):
    ctx.revoke("conformance: ensuring no owner")
    before = ctx.control_status().rejected_count
    pub = ctx.n.create_publisher(GripperSetpoint, "/setpoint/gripper", SP_QOS)
    ctx.spin(0.3)
    for _ in range(20):
        m = GripperSetpoint()
        m.position, m.speed, m.force = 0.0, 0.5, 0.5
        m.token = tok(ZERO_TOKEN)
        pub.publish(m)
        ctx.spin(0.02)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after <= before:
        return Result("", FAIL, "an untokened gripper setpoint was not counted as "
                                "rejected -- the Arbiter admitted it")
    return Result("", PASS, f"refused, rejected_count {before} -> {after}")


@REGISTRY.add(SEC, "enforced: a tokened setpoint moves the gripper",
              needs_motion=True, needs_mode="enforced")
def check_tokened_moves(ctx):
    token = ctx.acquire("conformance-gripper").token
    g0 = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if g0 is None or not g0.present:
        # SKIP, never PASS. The README's principle: a skip is reported, never silently
        # counted as a pass. SKIP is a first-class status the runner counts separately.
        return Result("", SKIP, "no gripper attached (present=false); nothing to command")
    pub = ctx.n.create_publisher(GripperSetpoint, "/setpoint/gripper", SP_QOS)
    ctx.spin(0.3)
    target = 0.3 if g0.position < 0.15 else 0.0
    end = time.time() + 4.0
    while time.time() < end:
        m = GripperSetpoint()
        m.position, m.speed, m.force = target, 0.3, 0.3
        m.token = tok(token)
        pub.publish(m)
        ctx.spin(0.02)
    g1 = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if abs(g1.position - g0.position) < 0.02:
        return Result("", FAIL, f"gripper did not move: {g0.position:.3f} -> "
                                f"{g1.position:.3f} commanding {target}")
    return Result("", PASS, f"moved {g0.position:.3f} -> {g1.position:.3f}")
