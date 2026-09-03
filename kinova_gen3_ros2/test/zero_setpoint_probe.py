#!/usr/bin/env python3
"""THROWAWAY probe: does a ZERO setpoint hold the arm?

One variable. joint_velocity is a passthrough -- no DLS solve, no null-space posture
term. ee_twist is the same actuator mode reached through the solve AND the posture bias.
Stream a hard zero on each and measure the drift.

  joint_velocity drifts ~0, ee_twist drifts  -> the posture term is driving the arm
  both drift                                 -> the actuator velocity servo is not holding
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from kinova_gen3_interfaces.msg import JointSetpoint, TwistSetpoint
from kinova_gen3_interfaces.srv import (
    AcquireControl,
    CloseStream,
    OpenStream,
    ReleaseControl,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

Q_REST = [0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57]
SECONDS = 6.0
SP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)


def norm(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class Probe(Node):
    def __init__(self):
        super().__init__("zero_setpoint_probe")
        self.q = None
        self.create_subscription(
            JointState,
            "/joint_states",
            lambda m: setattr(self, "q", list(m.position)),
            SENSOR_QOS,
        )
        self.pub_jv = self.create_publisher(
            JointSetpoint, "/setpoint/joint_velocity", SP_QOS
        )
        self.pub_tw = self.create_publisher(TwistSetpoint, "/setpoint/twist", SP_QOS)
        self.token = [0] * 16

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def call(self, t, name, req):
        cli = self.create_client(t, name)
        if not cli.wait_for_service(timeout_sec=10.0):
            sys.exit(f"service {name} never appeared")
        fut = cli.call_async(req)
        end = time.time() + 10.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result()

    def run_case(self, controller):
        print(f"\n=== {controller} : streaming a hard ZERO for {SECONDS:.0f}s ===")
        self.spin(0.5)
        if self.q is None:
            sys.exit("no /joint_states")
        q0 = list(self.q)
        r = self.call(
            OpenStream,
            "open_stream",
            OpenStream.Request(
                controller=controller, timeout_s=0.5, token=[int(x) for x in self.token]
            ),
        )
        if not r.accepted:
            print(f"  open refused: {r.message}")
            return None
        print(
            f"  open on {list(r.channels)}; |q0 - q_rest| = {norm(q0, Q_REST):.3f} rad"
        )

        end = time.time() + SECONDS
        nxt = time.time()
        while time.time() < end:
            if controller == "joint_velocity":
                self.pub_jv.publish(
                    JointSetpoint(values=[0.0] * 7, token=[int(x) for x in self.token])
                )
            else:
                self.pub_tw.publish(
                    TwistSetpoint(twist=Twist(), token=[int(x) for x in self.token])
                )
            if time.time() >= nxt:
                print(
                    f"    t={SECONDS - (end - time.time()):4.1f}s  "
                    f"moved={norm(self.q, q0):.4f} rad   "
                    f"|q-q_rest|={norm(self.q, Q_REST):.3f}"
                )
                nxt = time.time() + 1.0
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.01)

        q1 = list(self.q)
        self.call(
            CloseStream,
            "close_stream",
            CloseStream.Request(token=[int(x) for x in self.token]),
        )
        moved = norm(q1, q0)
        print(f"  RESULT {controller}: moved {moved:.4f} rad in {SECONDS:.0f}s")
        print(f"    per joint: {' '.join(f'{b - a:+.3f}' for a, b in zip(q0, q1))}")
        print(
            f"    |q-q_rest| {norm(q0, Q_REST):.3f} -> {norm(q1, Q_REST):.3f}"
            f"   ({'TOWARD q_rest' if norm(q1, Q_REST) < norm(q0, Q_REST) else 'AWAY from q_rest'})"
        )
        return moved


def main():
    rclpy.init()
    n = Probe()
    r = n.call(
        AcquireControl, "acquire_control", AcquireControl.Request(owner_id="zero_probe")
    )
    if not r.accepted:
        sys.exit(f"acquire refused: {r.message}")
    n.token = [int(x) for x in r.token]
    print(f"owned, generation={r.generation}")

    results = {}
    try:
        for c in ("joint_velocity", "ee_twist"):
            results[c] = n.run_case(c)
            n.spin(1.5)  # settle between sessions
    finally:
        print("\n=== verdict ===")
        jv, tw = results.get("joint_velocity"), results.get("ee_twist")
        if jv is not None and tw is not None:
            if tw > 5 * max(jv, 1e-4):
                print(
                    f"  joint_velocity held ({jv:.4f} rad), ee_twist drifted ({tw:.4f} rad)"
                )
                print(
                    "  => the DLS solve / null-space posture term is driving the arm."
                )
                print("     The actuator velocity servo is NOT at fault.")
            elif jv > 0.05 and tw > 0.05:
                print(f"  BOTH drifted (jv={jv:.4f}, twist={tw:.4f} rad)")
                print(
                    "  => a zero velocity command does not hold: the servo is not tracking zero."
                )
            else:
                print(
                    f"  both held (jv={jv:.4f}, twist={tw:.4f} rad) -- not reproduced here."
                )
        n.call(
            ReleaseControl,
            "release_control",
            ReleaseControl.Request(token=[int(x) for x in n.token]),
        )
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
