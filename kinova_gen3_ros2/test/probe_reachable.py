#!/usr/bin/env python3
"""Ask the planner which candidate poses are feasible. Plans only; never moves.

Stop guessing coordinates. This walks a ladder of poses, asks cuRobo to plan to
each, and prints which succeeded. It calls /rammp_curobo/plan_to_pose directly
rather than GoToEEPose, because planning never commands the arm -- the planner
node says so itself on startup ("This node never moves the arm; execution is the
caller's"). Nothing here executes a trajectory.

Use it to pick the endpoints for scenario_approach.py / scenario_carry_level.py
instead of discovering IK_FAIL one run at a time.

    python3 probe_reachable.py --z 0.45 0.40 0.35 0.30 0.25 0.20
    python3 probe_reachable.py --x 0.35 0.40 0.45 --z 0.45 0.30
    python3 probe_reachable.py --quat 1 0 0 0        # a different grasp pose

A note on frames, because it bites: the goal is cuRobo's `tool_frame` (the
fingertip midpoint, from NVIDIA's bundled Gen3 model). Our own /ee_state topic
publishes `gen3_end_effector_link` from OUR urdf, which sits about 12 cm behind
it along the tool axis. The two are different frames from different models, so a
number from one cannot be compared with a number from the other. Everything here
is in the planner's frame.
"""

import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rammp_curobo_interfaces.action import PlanToPose
    from rammp_curobo_interfaces.msg import ApproachVia, PoseAxisLock
    from sensor_msgs.msg import JointState

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# /joint_states carries EIGHT entries -- the seven arm joints plus the gripper's
# robotiq_85_left_knuckle_joint. The planner wants exactly the seven, so they are
# picked BY NAME rather than by slicing the first seven, which only works by
# luck of the current publish order.
ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]

# Gripper level, approach axis horizontal facing forward (+90 deg about Y).
GRIPPER_LEVEL = [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]


def _normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {q}")
    return [c / n for c in q]


if _HAVE_ROS:

    class Probe(Node):
        def __init__(self):
            super().__init__("probe_reachable")
            self.client = ActionClient(self, PlanToPose, "/rammp_curobo/plan_to_pose")
            self.joints = None
            self.last_traj = None
            self.create_subscription(
                JointState, "/joint_states", self._on_js, qos_profile_sensor_data
            )

        def _on_js(self, msg):
            by_name = dict(zip(msg.name, msg.position))
            if all(j in by_name for j in ARM_JOINTS):
                self.joints = [by_name[j] for j in ARM_JOINTS]

        def wait_for_joints(self, timeout=5.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end and self.joints is None:
                rclpy.spin_once(self, timeout_sec=0.05)
            return self.joints is not None

        def feasible(self, pos, quat, timeout=20.0, approach=None, locks=()):
            """(ok, message, seconds). Planning only -- the arm does not move.

            `approach`/`locks` go straight to the PLANNER, skipping the arm node.
            That is the point: if a via sent from here changes nothing and is
            never refused, the planner is ignoring it; if it behaves here but not
            through GoToEEPose, the arm node is dropping it. Same question, two
            halves, and only a direct probe separates them.
            """
            g = PlanToPose.Goal()
            g.target.position.x, g.target.position.y, g.target.position.z = pos
            (
                g.target.orientation.x,
                g.target.orientation.y,
                g.target.orientation.z,
                g.target.orientation.w,
            ) = quat
            # REQUIRED -- an empty list is refused ("7 joint positions expected,
            # got 0"). Every probe plans from where the arm is right now.
            g.start_joints = list(self.joints)
            g.axis_lock = PoseAxisLock()
            g.axis_lock.reference_frame = PoseAxisLock.FRAME_BASE
            for name in locks:
                setattr(g.axis_lock, f"lock_{name}", True)
            g.approach_via = ApproachVia()
            if approach:
                g.approach_via.offset = approach[0]
                g.approach_via.axis = {"x": 0, "y": 1, "z": 2}[approach[1]]
                g.approach_via.at_fraction = approach[2]
            t0 = time.monotonic()
            fut = self.client.send_goal_async(g)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            gh = fut.result()
            if gh is None or not gh.accepted:
                return False, "planner rejected the goal", time.monotonic() - t0
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
            if not rf.done():
                return False, "planner never answered", time.monotonic() - t0
            res = rf.result().result
            self.last_traj = res.trajectory
            return res.success, res.message, time.monotonic() - t0


def describe_traj(traj):
    """Report what the driver will actually be able to do with this plan.

    The interface gate is all-or-nothing: message_mapping sets has_velocities
    only if EVERY point carries a full-width velocity vector, and without it
    TrajectoryExecutor::sample falls back to linear interpolation between the
    20 ms waypoints -- piecewise-constant velocity, a 50 Hz staircase.
    """
    pts = traj.points
    n = len(pts)
    if n == 0:
        print("    trajectory: EMPTY")
        return
    width = len(traj.joint_names) or len(pts[0].positions)
    with_v = sum(1 for p in pts if len(p.velocities) == width)
    with_a = sum(1 for p in pts if len(p.accelerations) == width)
    ts = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
    dts = [b - a for a, b in zip(ts, ts[1:])] or [0.0]
    print(
        "    trajectory: %d points over %.2fs, dt min=%.1f max=%.1f ms"
        % (n, ts[-1], min(dts) * 1000, max(dts) * 1000)
    )
    gate = "CUBIC HERMITE" if with_v == n else "LINEAR (a 50 Hz velocity staircase)"
    print(
        "    velocities on %d/%d points, accelerations on %d/%d  ->  driver "
        "interpolates: %s" % (with_v, n, with_a, n, gate)
    )
    if with_v not in (0, n):
        print(
            "    NOTE only %d of %d carry velocities — one short point silently"
            % (with_v, n)
        )
        print("    drops the WHOLE trajectory to linear.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--x", type=float, nargs="+", default=[0.45])
    ap.add_argument("--y", type=float, nargs="+", default=[0.0])
    ap.add_argument(
        "--z", type=float, nargs="+", default=[0.45, 0.40, 0.35, 0.30, 0.25, 0.20]
    )
    ap.add_argument(
        "--inspect-traj",
        action="store_true",
        help="report the returned trajectory's point count, dt spacing and "
        "whether EVERY point carries velocities -- the driver falls back to "
        "LINEAR interpolation unless all of them do, which is a 50 Hz velocity "
        "staircase and feels like stutter",
    )
    ap.add_argument(
        "--approach",
        nargs=3,
        metavar=("OFFSET", "AXIS", "AT_FRACTION"),
        help="send an ApproachVia straight to the planner, e.g. --approach 0.1 z 0.8",
    )
    ap.add_argument(
        "--lock",
        nargs="*",
        default=[],
        choices=["roll", "pitch", "yaw", "x", "y", "z"],
        metavar="AXIS",
        help="send a PoseAxisLock straight to the planner",
    )
    ap.add_argument(
        "--quat",
        type=float,
        nargs=4,
        metavar=("X", "Y", "Z", "W"),
        default=GRIPPER_LEVEL,
        help="tool orientation to test, xyzw (default: gripper level facing forward)",
    )
    args = ap.parse_args()

    quat = _normalize(args.quat)
    approach = None
    if args.approach:
        approach = (float(args.approach[0]), args.approach[1].lower(), float(args.approach[2]))
        if approach[1] not in ("x", "y", "z"):
            ap.error("--approach AXIS must be x, y or z")
    poses = [(x, y, z) for x in args.x for y in args.y for z in args.z]
    print(f"\n  probing {len(poses)} pose(s), quat xyzw = "
          f"[{', '.join(f'{c:.4f}' for c in quat)}]")
    if approach:
        print(f"  approach_via: {approach[0]} m along {approach[1]} at {approach[2]}")
    if args.lock:
        print(f"  axis_lock: {'+'.join(args.lock)} (base frame)")
    print("  PLANNING ONLY — the arm does not move.\n")

    if not _HAVE_ROS:
        print("  needs rclpy + rammp_curobo_interfaces on the path.\n")
        return 2

    rclpy.init()
    n = Probe()
    if not n.client.wait_for_server(timeout_sec=10.0):
        print("  /rammp_curobo/plan_to_pose not available — is the planner up,")
        print("  and is CYCLONEDDS_URI exported in this shell?\n")
        rclpy.shutdown()
        return 1
    if not n.wait_for_joints():
        print("  no /joint_states — the planner needs a start state and refuses")
        print("  an empty one. Is the arm node up?\n")
        rclpy.shutdown()
        return 1
    print("  start state: " + ", ".join(f"{q:+.3f}" for q in n.joints) + "\n")

    head = f"  {'x':>6} {'y':>6} {'z':>6}  {'ok':<4} {'s':>5}  reason"
    print(head)
    print("  " + "-" * (len(head) + 24))
    ok_list = []
    try:
        for pos in poses:
            ok, msg, secs = n.feasible(pos, quat, approach=approach, locks=args.lock)
            if ok:
                ok_list.append(pos)
            if args.inspect_traj and ok and n.last_traj is not None:
                describe_traj(n.last_traj)
            short = (msg or "").strip().replace("\n", " ")
            # The planner's IK_FAIL text is long and the same every time; the
            # status prefix is the part that differs.
            if len(short) > 60:
                short = short[:57] + "..."
            print(
                f"  {pos[0]:>6.3f} {pos[1]:>6.3f} {pos[2]:>6.3f}  "
                f"{'YES' if ok else 'no':<4} {secs:>5.2f}  {short}"
            )
    finally:
        rclpy.shutdown()

    print(f"\n  {len(ok_list)}/{len(poses)} feasible.")
    if not ok_list:
        print("  none — try a different --quat, or pull --x in toward the base.")
        print()
        return 3

    # Grouped by COLUMN. Reporting a bare list of feasible z values across
    # different x was worse than useless: it once printed "feasible z values at
    # x=0.25" when x=0.25 had no feasible pose at all, and proposed a descent
    # pair whose ends were at different x.
    columns = {}
    for x, y, z in ok_list:
        columns.setdefault((x, y), []).append(z)
    best = None
    for (x, y), zs in sorted(columns.items()):
        zs = sorted(zs)
        print(f"  x={x:.3f} y={y:.3f}:  feasible z {zs}")
        if len(zs) >= 2 and (best is None or zs[-1] - zs[0] > best[2]):
            best = (x, y, zs[-1] - zs[0], zs[-1], zs[0])
    if best:
        x, y, drop, hi, lo = best
        print(
            f"\n  widest descent in one column: x={x:.3f} y={y:.3f}, "
            f"A z={hi} -> B z={lo}  ({drop:.2f} m)"
        )
    else:
        print("\n  no column has two feasible heights, so there is no descent")
        print("  pair at this orientation — a carry needs two poses, not one.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
