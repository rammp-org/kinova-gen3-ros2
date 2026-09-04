"""Output surfaces: does the node actually publish what it claims, on the right QoS."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from kinova_gen3_interfaces.msg import EeState

from harness import FAIL, PASS, REGISTRY, SENSOR_QOS, Result

SEC = "state"


@REGISTRY.add(SEC, "/joint_states publishes seven named arm joints")
def check_joint_states(ctx):
    js = ctx.latest("joint_states", timeout=6.0, fresh=True)
    if js is None:
        return Result("", FAIL, "nothing on /joint_states (QoS is best-effort)")
    expected = [f"joint_{i}" for i in range(1, 8)]
    if list(js.name[:7]) != expected:
        return Result(
            "",
            FAIL,
            f"names are {list(js.name[:7])}, expected {expected}. "
            "robot_state_publisher matches by NAME; a mismatch here "
            "freezes TF silently.",
        )
    if not (len(js.position) >= 7 and len(js.velocity) >= 7 and len(js.effort) >= 7):
        return Result("", FAIL, "position/velocity/effort are not all populated")
    return Result("", PASS, f"7 joints, q[0]={js.position[0]:+.4f}")


@REGISTRY.add(SEC, "/ee_state publishes a plausible pose and twist")
def check_ee_state(ctx):
    ctx.sub(EeState, "ee_state", SENSOR_QOS)
    ee = ctx.latest("ee_state", timeout=6.0, fresh=True)
    if ee is None:
        return Result("", FAIL, "nothing on /ee_state")
    p = ee.pose.position
    r = ee.pose.orientation
    norm = math.sqrt(r.x**2 + r.y**2 + r.z**2 + r.w**2)
    if abs(norm - 1.0) > 1e-6:
        return Result(
            "", FAIL, f"orientation is not a unit quaternion (|q|={norm:.6f})"
        )
    reach = math.sqrt(p.x**2 + p.y**2 + p.z**2)
    # The Gen3's tool cannot be at the origin and cannot be 2 m away. A zero here means
    # ee_pose was never populated -- the exact bug /ee_state was added to fix.
    if not (0.1 < reach < 1.6):
        return Result("", FAIL, f"tool is {reach:.3f} m from base; implausible")
    return Result("", PASS, f"p=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) |p|={reach:.3f}m")


@REGISTRY.add(SEC, "/ee_state and /joint_states come from the same pump tick")
def check_state_consistency(ctx):
    ctx.sub(EeState, "ee_state", SENSOR_QOS)
    js = ctx.latest("joint_states", timeout=6.0, fresh=True)
    ee = ctx.latest("ee_state", timeout=6.0, fresh=True)
    if js is None or ee is None:
        return Result("", FAIL, "missing one of the two topics")
    dt = abs(
        (js.header.stamp.sec - ee.header.stamp.sec)
        + (js.header.stamp.nanosec - ee.header.stamp.nanosec) * 1e-9
    )
    # Both are stamped from the same publish_state call, so consecutive samples should
    # be within a pump period (~10 ms). Anything larger means they are not paired.
    if dt > 0.05:
        return Result("", FAIL, f"stamps differ by {dt*1000:.1f} ms; not the same tick")
    return Result("", PASS, f"stamps within {dt*1000:.2f} ms")


@REGISTRY.add(SEC, "/control_status is latched and self-consistent")
def check_control_status(ctx):
    cs = ctx.control_status()
    if cs is None:
        return Result("", FAIL, "nothing on /control_status (needs transient_local)")
    if cs.owned and not cs.owner_id:
        return Result("", FAIL, "owned=true but owner_id is empty")
    if not cs.owned and cs.owner_id:
        return Result("", FAIL, f"owned=false but owner_id='{cs.owner_id}'")
    return Result(
        "",
        PASS,
        f"mode={'enforced' if cs.arbitration_enabled else 'disabled'} "
        f"owned={cs.owned} estopped={cs.estopped} gen={cs.generation}",
    )


@REGISTRY.add(SEC, "/stream_status is latched and reports core's view")
def check_stream_status(ctx):
    ss = ctx.stream_status()
    if ss is None:
        return Result("", FAIL, "nothing on /stream_status")
    if not ss.open and ss.controller:
        return Result("", FAIL, f"closed but still labelled '{ss.controller}'")
    return Result(
        "",
        PASS,
        f"open={ss.open} controller='{ss.controller}' " f"rejected={ss.rejected_count}",
    )


def _level(s):
    """DiagnosticStatus.level is a byte in rclpy; normalise to int."""
    return (
        int.from_bytes(s.level, "big") if isinstance(s.level, bytes) else int(s.level)
    )


@REGISTRY.add(SEC, "/diagnostics carries all three REP 107 tasks")
def check_diagnostics(ctx):
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    seen = {}
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    # ArbitrationServer, Ros2Backend and GripperServer each own a diagnostic_updater,
    # so they publish SEPARATE DiagnosticArrays, one task apiece. Reading a single
    # message finds one task and wrongly concludes the others are missing -- collect
    # over a window instead.
    sub = ctx.n.create_subscription(
        DiagnosticArray,
        "/diagnostics",
        lambda m: [seen.__setitem__(s.name, s) for s in m.status],
        qos,
    )
    ctx.spin(4.0)  # all three updaters tick at 1 Hz
    ctx.n.destroy_subscription(sub)

    if not seen:
        return Result("", FAIL, "nothing on /diagnostics")
    want = ("Arbitration", "Arm", "Gripper")
    missing = [w for w in want if not any(w in n for n in seen)]
    if missing:
        return Result("", FAIL, f"missing task(s) {missing}; saw {sorted(seen)}")
    arm = next(s for n, s in seen.items() if "Arm" in n)
    if _level(arm) == DiagnosticStatus.STALE:
        return Result("", FAIL, "Arm task is STALE -- no feedback has reached the node")
    return Result("", PASS, f"tasks={sorted(seen)} arm_level={_level(arm)}")
