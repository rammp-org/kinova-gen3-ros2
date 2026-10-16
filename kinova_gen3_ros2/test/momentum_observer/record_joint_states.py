#!/usr/bin/env python3
"""Record /joint_states (and the published EE wrench) to a CSV, with tap markers.

Passive: subscribes only. Run it beside any move you already make, for example
send_joint_tour.py, and press Enter at the moment someone taps the arm. Each
Enter stamps a marker on the next row so analyze.py knows where the real pushes
were.

  python3 record_joint_states.py --out run.csv --duration 60

Columns: t, q1..q7 (rad), qd1..qd7 (rad/s), tau1..tau7 (N.m, measured),
fx fy fz tx ty tz (the /ee_state wrench core publishes, NaN if absent), marker.
"""

import argparse
import csv
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)
N = 7


class Recorder(Node):
    def __init__(self, out, duration):
        super().__init__("momentum_observer_recorder")
        self.f = open(out, "w", newline="")  # noqa: SIM115  (closed in close())
        self.w = csv.writer(self.f)
        self.w.writerow(
            ["t"]
            + [f"q{i}" for i in range(1, N + 1)]
            + [f"qd{i}" for i in range(1, N + 1)]
            + [f"tau{i}" for i in range(1, N + 1)]
            + ["fx", "fy", "fz", "tx", "ty", "tz", "marker"]
        )
        self.t0 = None
        self.deadline = time.monotonic() + duration if duration > 0 else None
        self.rows = 0
        self.markers = 0
        self.pending_marker = False
        self.wrench = [math.nan] * 6
        self.create_subscription(JointState, "/joint_states", self.on_js, SENSOR_QOS)
        try:
            from rammp_arm_interfaces.msg import EeState

            self.create_subscription(EeState, "/ee_state", self.on_ee, SENSOR_QOS)
        except ImportError:
            self.get_logger().warning(
                "rammp_arm_interfaces not found; wrench columns stay NaN"
            )

    def on_ee(self, m):
        w = m.wrench
        self.wrench = [
            w.force.x,
            w.force.y,
            w.force.z,
            w.torque.x,
            w.torque.y,
            w.torque.z,
        ]

    def on_js(self, m):
        if len(m.position) < N or len(m.effort) < N:
            return
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        marker = 1 if self.pending_marker else 0
        if marker:
            self.pending_marker = False
            self.markers += 1
            print(f"marker {self.markers} at t={stamp - self.t0:.3f}", flush=True)
        self.w.writerow(
            [f"{stamp - self.t0:.4f}"]
            + [f"{v:.6f}" for v in m.position[:N]]
            + [f"{v:.6f}" for v in m.velocity[:N]]
            + [f"{v:.4f}" for v in m.effort[:N]]
            + [f"{v:.4f}" for v in self.wrench]
            + [marker]
        )
        self.rows += 1

    def close(self):
        self.f.close()


def stdin_markers(rec):
    for _ in sys.stdin:
        rec.pending_marker = True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True)
    ap.add_argument(
        "--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl-C"
    )
    args = ap.parse_args()
    rclpy.init()
    rec = Recorder(args.out, args.duration)
    threading.Thread(target=stdin_markers, args=(rec,), daemon=True).start()
    print("recording; press Enter at each tap, Ctrl-C to stop", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(rec, timeout_sec=0.05)
            if rec.deadline and time.monotonic() >= rec.deadline:
                break
    except KeyboardInterrupt:
        pass
    rec.close()
    print(f"{args.out}: {rec.rows} rows, {rec.markers} markers")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
