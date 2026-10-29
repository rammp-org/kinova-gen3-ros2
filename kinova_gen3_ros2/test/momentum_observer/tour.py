#!/usr/bin/env python3
"""Drive the arm around a short joint-space tour so there is motion to tap during.

Sequential GoToJointConfig goals (cuRobo plans and collision-checks each leg),
using only the fields the v1.1.0 interfaces carry. Dry run by default; --go
moves the arm. Attended, e-stop in hand.

  python3 tour.py                      # print the tour
  python3 tour.py --go --loops 2       # move
  python3 tour.py --go --speed 0.6     # slower planner speed scale
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rammp_arm_interfaces.action import GoToJointConfig

HOME = [0.0, 0.262, 3.142, -2.269, 0.0, 0.960, 1.571]
OFFSETS = [
    [0.5, 0.25, 0.0, 0.35, 0.0, -0.25, 0.0],
    [-0.5, 0.25, 0.0, 0.35, 0.0, -0.25, 0.0],
    [0.0, -0.15, 0.0, 0.0, 0.5, 0.30, 0.6],
    [0.0] * 7,
]
RESULT = {0: "SUCCESSFUL", -1: "INVALID_GOAL", -4: "PATH_TOLERANCE_VIOLATED",
          -6: "PREEMPTED", -7: "PLANNING_FAILED", -8: "NOT_AUTHORIZED", -9: "HALTED"}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0, help="speed_scale on each goal")
    args = ap.parse_args()
    tour = [[h + d for h, d in zip(HOME, off)] for off in OFFSETS]
    for i, wp in enumerate(tour):
        print(f"leg {i}: " + " ".join(f"{v:+.3f}" for v in wp))
    if not args.go:
        print("dry run; pass --go to move")
        return 0
    rclpy.init()
    node = Node("momentum_observer_tour")
    client = ActionClient(node, GoToJointConfig, "go_to_joint_config")
    if not client.wait_for_server(timeout_sec=5.0):
        print("go_to_joint_config unavailable: is the arm node up?")
        return 1
    rc = 0
    try:
        for loop in range(args.loops):
            for i, wp in enumerate(tour):
                goal = GoToJointConfig.Goal()
                goal.target_joints = [float(v) for v in wp]
                goal.sender_id = "momentum_observer_tour"
                goal.speed_scale = args.speed
                send = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, send)
                gh = send.result()
                if gh is None or not gh.accepted:
                    print(f"loop {loop} leg {i}: rejected")
                    return 1
                res = gh.get_result_async()
                rclpy.spin_until_future_complete(node, res)
                r = res.result().result
                print(f"loop {loop} leg {i}: {RESULT.get(r.error_code, r.error_code)} {r.error_string}")
                if r.error_code != 0:
                    rc = 1
                    break
            if rc:
                break
    except KeyboardInterrupt:
        print("interrupted")
        rc = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
