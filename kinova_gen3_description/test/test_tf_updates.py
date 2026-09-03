#!/usr/bin/env python3
"""TF must actually MOVE when the joints do.

The defect this package fixes was silent: the driver published joint_1..joint_7 while
the URDF declared gen3_joint_1..gen3_joint_7, so robot_state_publisher matched nothing
and held TF at the default configuration forever. Nothing errored. A test that only
checked "a transform exists" would have passed the entire time -- so this asserts the
transform CHANGES.

Two things this test learned the hard way, both worth keeping:

  * `ros2 topic pub` is useless as a source here. It sends header.stamp = 0, and
    robot_state_publisher drops joint states that do not advance in time, so only the
    very first message is ever processed.

  * robot_state_publisher DOES derive <mimic> joint transforms from the joint they
    mimic -- verified 2026-09-03 with a two-joint URDF: publishing only the driver
    joint moved the mimic link by exactly -0.6 rad for a +0.6 rad drive. The
    articulated model has 13 movable joints but only EIGHT independent ones (seven arm
    plus robotiq_85_left_knuckle_joint); the driver publishes all eight as of the
    gripper tier, so the description launch now serves the articulated model by
    default.

Run against a live `ros2 launch kinova_gen3_description description.launch.py`:
    python3 test_tf_updates.py
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

BASE, TOOL = "base_link", "end_effector_link"
JOINTS = [f"joint_{i}" for i in range(1, 8)]


class Prober(Node):
    def __init__(self):
        super().__init__("tf_update_test")
        self.buf = Buffer()
        self._listener = TransformListener(self.buf, self)
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.angle = 0.0
        # A timer rather than a publish loop: the TF listener needs the executor, and
        # a tight publish loop starves it.
        self.create_timer(0.02, self._tick)

    def _tick(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = JOINTS
        m.position = [0.0, self.angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._pub.publish(m)

    def settle_at(self, angle, seconds=3.0):
        """Hold a pose, then read the transform."""
        self.angle = angle
        deadline = time.time() + seconds
        last = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.buf.lookup_transform(
                    BASE, TOOL, rclpy.time.Time()).transform.translation
                last = (t.x, t.y, t.z)
            except Exception:
                pass
        return last


def main() -> int:
    rclpy.init()
    node = Prober()
    try:
        first = node.settle_at(0.0)
        if first is None:
            print(f"FAIL: no transform {BASE} -> {TOOL}. Is robot_state_publisher "
                  "running, and is it serving the 7-DOF model?")
            return 1

        second = node.settle_at(math.pi / 4)
        if second is None:
            print("FAIL: transform vanished after the second pose")
            return 1

        moved = max(abs(a - b) for a, b in zip(first, second))
        print(f"  joint_2 = 0     -> {tuple(round(v, 4) for v in first)}")
        print(f"  joint_2 = pi/4  -> {tuple(round(v, 4) for v in second)}")
        print(f"  max delta       =  {moved:.4f} m")
        if moved <= 0.01:
            print("FAIL: TF did not move. This is the frozen-TF defect: the joint names "
                  "in /joint_states do not match the URDF.")
            return 1
        print("PASS: TF follows /joint_states")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
