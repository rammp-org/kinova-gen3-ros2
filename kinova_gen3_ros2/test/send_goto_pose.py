#!/usr/bin/env python3
"""Send a GoToEEPose goal (base_link target) and print feedback + result.

The operator supplies a base_link tool pose. Pick a pose near the current tool
pose for a safe local move; cuRobo plans collision-free from the live /joint_states.
"""
import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from kinova_gen3_interfaces.action import GoToEEPose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                    help="target tool position in base_link (metres)")
    ap.add_argument("--quat", type=float, nargs=4, required=True,
                    metavar=("X", "Y", "Z", "W"), help="target tool orientation xyzw")
    ap.add_argument("--sender-id", default="send_goto_pose")
    args = ap.parse_args()

    rclpy.init()
    node = Node("send_goto_pose")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        return 1

    goal = GoToEEPose.Goal()
    goal.target.header.frame_id = "base_link"
    goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = args.pos
    (goal.target.pose.orientation.x, goal.target.pose.orientation.y,
     goal.target.pose.orientation.z, goal.target.pose.orientation.w) = args.quat
    goal.sender_id = args.sender_id

    def on_fb(fb):
        f = fb.feedback
        node.get_logger().info(
            f"[{f.phase}] planner_state='{f.planner_state}' frac={f.fraction_complete:.2f}")

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        node.get_logger().error("goal REJECTED (check frame_id == base_link)")
        return 2
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    node.get_logger().info(f"result error_code={res.error_code} msg='{res.error_string}'")
    rclpy.shutdown()
    return 0 if res.error_code == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
