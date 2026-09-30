#!/usr/bin/env python3
"""Send a GoToEEPose goal (base_link target) and print feedback + result.

The operator supplies a base_link tool pose. Pick a pose near the current tool
pose for a safe local move; cuRobo plans collision-free from the live /joint_states.

Optional constraints, one at a time — this is the single-shot companion to
sweep_constraints.py (which runs the whole matrix) and send_goto_pose_tour.py
(which runs a lap):

    --speed-scale S        pace the execution; same path, longer clock
    --lock roll pitch      hold tool-pose components for the whole move
    --approach-distance D  come in along --approach-axis over the last stretch

A lock holds a component AT THE GOAL'S VALUE, so the current pose must already
match the target on it, and an approach implies holding the other five — so both
can be refused before planning starts, with the reason in the node's log.
"""

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rammp_arm_interfaces.action import GoToEEPose
from rammp_arm_interfaces.msg import ApproachOffset, ToolAxisLock

_LOCKABLE = ["roll", "pitch", "yaw", "x", "y", "z"]
_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
_FRAMES = {"base": 0, "goal": 1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--pos",
        type=float,
        nargs=3,
        required=True,
        metavar=("X", "Y", "Z"),
        help="target tool position in base_link (metres)",
    )
    ap.add_argument(
        "--quat",
        type=float,
        nargs=4,
        required=True,
        metavar=("X", "Y", "Z", "W"),
        help="target tool orientation xyzw",
    )
    ap.add_argument("--sender-id", default="send_goto_pose")
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="execution pace; 1.0 = as planned. Refused outside [0.01, 1.0].",
    )
    ap.add_argument(
        "--lock",
        nargs="*",
        default=[],
        choices=_LOCKABLE,
        metavar="AXIS",
        help=f"tool-pose components to hold: {' '.join(_LOCKABLE)}",
    )
    ap.add_argument(
        "--frame",
        choices=sorted(_FRAMES),
        default="base",
        help="frame the locks and the approach are measured in (default base)",
    )
    ap.add_argument(
        "--approach-distance",
        type=float,
        default=0.0,
        help="metres back from the goal to approach from; 0 = no approach",
    )
    ap.add_argument("--approach-axis", choices=sorted(_AXIS_INDEX), default="z")
    ap.add_argument("--at-fraction", type=float, default=0.8)
    args = ap.parse_args()

    # Same bounds the node enforces, refused rather than clamped — catching it
    # here saves a round trip and gives a reason, which a rejection cannot carry.
    if not 0.01 <= args.speed_scale <= 1.0:
        ap.error(f"--speed-scale must be in [0.01, 1.0]; got {args.speed_scale}")
    if args.approach_distance < 0.0:
        ap.error("--approach-distance must be >= 0")
    if args.approach_distance and not 0.0 < args.at_fraction < 1.0:
        ap.error("--at-fraction must be strictly inside (0, 1)")

    rclpy.init()
    node = Node("send_goto_pose")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        return 1

    goal = GoToEEPose.Goal()
    goal.target.header.frame_id = "base_link"
    (
        goal.target.pose.position.x,
        goal.target.pose.position.y,
        goal.target.pose.position.z,
    ) = args.pos
    (
        goal.target.pose.orientation.x,
        goal.target.pose.orientation.y,
        goal.target.pose.orientation.z,
        goal.target.pose.orientation.w,
    ) = args.quat
    goal.sender_id = args.sender_id
    goal.speed_scale = args.speed_scale

    goal.axis_lock = ToolAxisLock()
    goal.axis_lock.reference_frame = _FRAMES[args.frame]
    for name in args.lock:
        setattr(goal.axis_lock, f"lock_{name}", True)

    goal.approach_offset = ApproachOffset()
    if args.approach_distance:
        goal.approach_offset.distance = args.approach_distance
        goal.approach_offset.axis = _AXIS_INDEX[args.approach_axis]
        goal.approach_offset.at_fraction = args.at_fraction

    def on_fb(fb):
        f = fb.feedback
        node.get_logger().info(
            f"[{f.phase}] planner_state='{f.planner_state}' frac={f.fraction_complete:.2f}"
        )

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        node.get_logger().error("goal REJECTED (check frame_id == base_link)")
        return 2
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    node.get_logger().info(
        f"result error_code={res.error_code} msg='{res.error_string}'"
    )
    rclpy.shutdown()
    return 0 if res.error_code == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
