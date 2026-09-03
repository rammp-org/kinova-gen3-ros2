#!/usr/bin/env python3
"""Send a GoToJointConfig or GoToPreset goal and report the result.

Dry-run by default: prints the goal and exits without sending. Pass --go to
actually execute. On real hardware the planned trajectory runs at full planner
speed -- read docs/on-robot-runbook.md and keep the e-stop in hand first.

  send_goto_joints.py --preset home --go
  send_goto_joints.py --joints 0 0.26 3.14 -2.27 0 0.96 1.57 --go
  send_goto_joints.py --delta 0.1 --joint 6 --go     # relative to current q
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from kinova_gen3_interfaces.action import GoToJointConfig, GoToPreset

RESULT_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
}


class Sender(Node):
    def __init__(self):
        super().__init__("send_goto_joints")
        self.q = None
        # The node publishes /joint_states with sensor QoS (BEST_EFFORT); a
        # default RELIABLE subscription silently receives nothing from it.
        self.create_subscription(
            JointState, "/joint_states", self._on_js, qos_profile_sensor_data
        )

    def _on_js(self, msg):
        if self.q is None and len(msg.position) >= 7:
            self.q = list(msg.position[:7])

    def wait_for_q(self, timeout_s=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.q is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q

    def send(self, action_type, name, goal, feedback_phases):
        client = ActionClient(self, action_type, name)
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server {name} unavailable")
            return None

        def on_fb(fb):
            f = fb.feedback
            key = (f.phase, f.planner_state)
            if key not in feedback_phases:
                feedback_phases.add(key)
                self.get_logger().info(f"  phase={f.phase} planner={f.planner_state}")

        gh_future = client.send_goal_async(goal, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(self, gh_future)
        gh = gh_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("goal REJECTED by the server")
            return None
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result


def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--preset", help="named preset (GoToPreset)")
    g.add_argument("--joints", nargs=7, type=float, help="absolute 7 joints, rad")
    g.add_argument("--delta", type=float, help="offset from current q, rad")
    p.add_argument("--joint", type=int, default=6, help="joint index for --delta")
    p.add_argument("--go", action="store_true", help="actually send (default: dry run)")
    args = p.parse_args()

    rclpy.init()
    node = Sender()
    try:
        if args.preset:
            goal, action_type, name = GoToPreset.Goal(), GoToPreset, "go_to_preset"
            goal.preset_name = args.preset
            desc = f"GoToPreset preset_name='{args.preset}'"
        else:
            if args.joints:
                target = list(args.joints)
            else:
                q = node.wait_for_q()
                if q is None:
                    node.get_logger().error(
                        "no /joint_states — is kinova_gen3_node up?"
                    )
                    return 2
                target = list(q)
                target[args.joint] += args.delta
            goal, action_type, name = (
                GoToJointConfig.Goal(),
                GoToJointConfig,
                "go_to_joint_config",
            )
            goal.target_joints = target
            desc = (
                "GoToJointConfig target=["
                + ", ".join(f"{v:+.3f}" for v in target)
                + "]"
            )

        goal.sender_id = "send_goto_joints"
        print(desc)
        if not args.go:
            print("dry run — pass --go to execute")
            return 0

        result = node.send(action_type, name, goal, set())
        if result is None:
            return 1
        label = RESULT_CODES.get(result.error_code, str(result.error_code))
        print(f"result: {label} ({result.error_code}) {result.error_string}")
        return 0 if result.error_code == 0 else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
