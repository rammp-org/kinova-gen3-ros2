#!/usr/bin/env python3
"""Sweep every streaming surface: open (gains at open), setpoint flow, deadline
close, hold, explicit close -- one PASS/FAIL line per controller, plus the
contract refusals (gains on a stiff stream, cartesian_impedance, unknown).

Safe by default: setpoints are the measured q / zero velocity, so nothing
moves. --go adds small motions with tracking asserts (j7 +0.1 rad for the
position streams, 0.05 rad/s for velocity, 0.01 m/s +z for twist).

The push tests stay manual -- this script proves the plumbing so your hands
are free to prove the compliance.

  stream_surface_sweep.py            # zero-motion mechanics pass, live arm OK
  stream_surface_sweep.py --go       # small motions + tracking asserts
  stream_surface_sweep.py --profile soft --go
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState

from rammp_arm_interfaces.msg import JointSetpoint, StreamStatus, TwistSetpoint
from rammp_arm_interfaces.srv import CloseStream, ListControllers, OpenStream

from gains_cli import PROFILES

TIMEOUT_S = 0.5  # stream deadline; generous for a 50 Hz publisher
STREAM_S = 2.0  # how long each surface streams
RATE_S = 0.02  # 50 Hz


class Sweep(Node):
    def __init__(self):
        super().__init__("stream_surface_sweep")
        self.q = None
        self.status = None
        self.create_subscription(
            JointState, "/joint_states", self._on_js, qos_profile_sensor_data
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(StreamStatus, "/stream_status", self._on_st, latched)
        self.cli_open = self.create_client(OpenStream, "/open_stream")
        self.cli_close = self.create_client(CloseStream, "/close_stream")
        self.cli_list = self.create_client(ListControllers, "/list_controllers")
        # Setpoint topics are best-effort KeepLast(1); sensor-data QoS matches.
        self.pub_jp = self.create_publisher(
            JointSetpoint, "/setpoint/joint_position", qos_profile_sensor_data
        )
        self.pub_jv = self.create_publisher(
            JointSetpoint, "/setpoint/joint_velocity", qos_profile_sensor_data
        )
        self.pub_tw = self.create_publisher(
            TwistSetpoint, "/setpoint/twist", qos_profile_sensor_data
        )

    def _on_js(self, m):
        if len(m.position) >= 7:
            self.q = list(m.position[:7])

    def _on_st(self, m):
        self.status = m

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def spin_for(self, sec):
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_q(self):
        t0 = time.monotonic()
        while self.q is None and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None

    def open(self, controller, profile_byte):
        req = OpenStream.Request()
        req.controller = controller
        req.timeout_s = TIMEOUT_S
        req.gains.profile = profile_byte
        return self.call(self.cli_open, req)

    def close(self):
        return self.call(self.cli_close, CloseStream.Request())


def check(results, label, ok, detail=""):
    results.append((label, bool(ok)))
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{': ' + detail if detail else ''}")


def run_surface(node, results, name, pub, make_msg, profile_byte, moved_expect):
    print(f"[{name}]")
    r = node.open(name, profile_byte)
    if r is None or not r.accepted:
        check(results, f"{name} open", False, getattr(r, "message", "no response"))
        return
    check(results, f"{name} open", True, f"channels={list(r.channels)}")
    q_start = list(node.q)
    # rejected_count is cumulative across the driver's lifetime, not per
    # session -- assert the DELTA over this surface, or a stale rejection from
    # an earlier run fails every later surface.
    node.spin_for(0.3)  # let the post-open status land
    rc_base = node.status.rejected_count if node.status else 0

    t0 = time.monotonic()
    while time.monotonic() - t0 < STREAM_S:
        pub.publish(make_msg(q_start))
        rclpy.spin_once(node, timeout_sec=RATE_S)
    st = node.status
    check(
        results,
        f"{name} stays open while streaming",
        st is not None and st.open and st.controller == name,
    )
    check(
        results,
        f"{name} no setpoints rejected this session",
        st is not None and st.rejected_count - rc_base == 0,
        f"delta {st.rejected_count - rc_base if st else '?'}",
    )
    if moved_expect > 0.0:
        moved = abs(node.q[6] - q_start[6])
        check(
            results,
            f"{name} tracked (j7 moved >= {moved_expect:.3f})",
            moved >= moved_expect,
            f"moved {moved:.4f} rad",
        )

    # Stop publishing: the deadline must tear the session down, and the arm
    # must hold at measured q (no creep, no sag).
    node.spin_for(TIMEOUT_S + 0.7)
    check(
        results,
        f"{name} deadline closed the session",
        node.status is not None and not node.status.open,
    )
    q_hold = list(node.q)
    node.spin_for(1.0)
    drift = max(abs(a - b) for a, b in zip(node.q, q_hold))
    check(results, f"{name} holds after close (drift < 0.02)", drift < 0.02,
          f"drift {drift:.4f} rad")
    node.close()  # no-op if the deadline already closed it


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--go", action="store_true", help="small motions + tracking asserts")
    ap.add_argument(
        "--profile",
        choices=sorted(PROFILES),
        default="default",
        help="gains profile for the compliant streams",
    )
    args = ap.parse_args()
    prof = PROFILES[args.profile]
    d_pos = 0.10 if args.go else 0.0  # j7 offset, rad
    v_j7 = 0.05 if args.go else 0.0  # rad/s
    v_z = 0.01 if args.go else 0.0  # m/s

    rclpy.init()
    node = Sweep()
    results = []
    try:
        for cli in (node.cli_open, node.cli_close, node.cli_list):
            if not cli.wait_for_service(timeout_sec=5.0):
                print(f"ERROR: {cli.srv_name} unavailable (node up?)")
                return 1
        if not node.wait_q():
            print("ERROR: no /joint_states")
            return 1

        avail = {
            c.name: c.available
            for c in node.call(node.cli_list, ListControllers.Request()).controllers
        }
        for want in ("joint_velocity_impedance", "ee_twist_impedance"):
            check(results, f"{want} listed available", avail.get(want, False))
        check(
            results,
            "cartesian_impedance NOT available",
            not avail.get("cartesian_impedance", False),
        )

        def jp(target_j7_delta):
            def make(q_start):
                m = JointSetpoint()
                m.values = list(q_start)
                m.values[6] = q_start[6] + target_j7_delta
                return m
            return make

        def jv(rate):
            def make(_):
                m = JointSetpoint()
                m.values = [0.0] * 6 + [rate]
                return m
            return make

        def tw(vz):
            def make(_):
                m = TwistSetpoint()
                m.twist.linear.z = vz
                return m
            return make

        half = 0.5  # accept >= half the commanded excursion as "tracked"
        surfaces = [
            ("joint_position", node.pub_jp, jp(d_pos), 0, d_pos * half),
            ("joint_impedance", node.pub_jp, jp(d_pos), prof, d_pos * half),
            ("joint_velocity", node.pub_jv, jv(v_j7), 0, v_j7 * STREAM_S * half),
            ("joint_velocity_impedance", node.pub_jv, jv(v_j7), prof,
             v_j7 * STREAM_S * half),
            ("ee_twist", node.pub_tw, tw(v_z), 0, 0.0),  # j7 isn't the twist axis
            ("ee_twist_impedance", node.pub_tw, tw(v_z), prof, 0.0),
        ]
        for name, pub, make, p, expect in surfaces:
            run_surface(node, results, name, pub, make, p, expect)

        print("[contract refusals]")
        r = node.open("joint_velocity", PROFILES["stiff"])
        check(results, "gains on a stiff stream refused",
              r is not None and not r.accepted, getattr(r, "message", ""))
        r = node.open("cartesian_impedance", 0)
        check(results, "cartesian_impedance refused",
              r is not None and not r.accepted, getattr(r, "message", ""))
        r = node.open("no_such_controller", 0)
        check(results, "unknown controller refused",
              r is not None and not r.accepted, getattr(r, "message", ""))

        failed = [l for l, ok in results if not ok]
        print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
        if failed:
            for l in failed:
                print(f"  FAILED: {l}")
        return 1 if failed else 0
    finally:
        node.close()  # belt and braces: never leave a session open
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
