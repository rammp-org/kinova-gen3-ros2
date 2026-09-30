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
    from rammp_curobo_interfaces.action import PlanToPose

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

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

        def feasible(self, pos, quat, timeout=20.0):
            """(ok, message, seconds). Planning only -- the arm does not move."""
            g = PlanToPose.Goal()
            g.target.position.x, g.target.position.y, g.target.position.z = pos
            (
                g.target.orientation.x,
                g.target.orientation.y,
                g.target.orientation.z,
                g.target.orientation.w,
            ) = quat
            g.start_joints = []          # empty => cuRobo reads our /joint_states
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
            return res.success, res.message, time.monotonic() - t0


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
        "--quat",
        type=float,
        nargs=4,
        metavar=("X", "Y", "Z", "W"),
        default=GRIPPER_LEVEL,
        help="tool orientation to test, xyzw (default: gripper level facing forward)",
    )
    args = ap.parse_args()

    quat = _normalize(args.quat)
    poses = [(x, y, z) for x in args.x for y in args.y for z in args.z]
    print(f"\n  probing {len(poses)} pose(s), quat xyzw = "
          f"[{', '.join(f'{c:.4f}' for c in quat)}]")
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

    head = f"  {'x':>6} {'y':>6} {'z':>6}  {'ok':<4} {'s':>5}  reason"
    print(head)
    print("  " + "-" * (len(head) + 24))
    ok_list = []
    try:
        for pos in poses:
            ok, msg, secs = n.feasible(pos, quat)
            if ok:
                ok_list.append(pos)
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
    if ok_list:
        zs = sorted({p[2] for p in ok_list})
        print(f"  feasible z values at x={args.x[0]}: {zs}")
        if len(zs) >= 2:
            print(f"  a usable descent pair: A z={zs[-1]}  ->  B z={zs[0]}"
                  f"  ({zs[-1] - zs[0]:.2f} m)")
    else:
        print("  none — try a different --quat, or pull --x in toward the base.")
    print()
    return 0 if ok_list else 3


if __name__ == "__main__":
    sys.exit(main())
