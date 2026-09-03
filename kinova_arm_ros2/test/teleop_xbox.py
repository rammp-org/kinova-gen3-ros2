#!/usr/bin/env python3
"""QUICK AND DIRTY Xbox-pad teleop for the EE streaming controllers. Throwaway.

Not a test and not a supported tool -- a hand-flying rig to feel out ee_pose_position
and ee_twist on the arm. No deadman: sticks at centre command zero, and if this script
dies the stream's timeout_s expires and the driver safe-stops on its own.

    LS         linear X / Y        RS up-down   linear Z
    RS left-right  yaw             A            toggle POSE <-> TWIST
    B          quit

    python3 teleop_xbox.py --dry-run     # print setpoints, open no stream, touch nothing
"""
import argparse
import math
import os
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from kinova_arm_interfaces.msg import EeState, PoseSetpoint, TwistSetpoint
from kinova_arm_interfaces.srv import (AcquireControl, CloseStream, OpenStream,
                                       ReleaseControl)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

SETPOINT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)

# xpad axis/button numbers. LX LY LT RX RY RT / A B.
AX_LX, AX_LY, AX_RX, AX_RY = 0, 1, 3, 4
BTN_A, BTN_B = 0, 1
DEADZONE = 0.15
RATE_HZ = 100.0
LEASH_M = 0.10          # the integrated POSE target may never outrun measured by more


def tok(t):
    """rclpy hands uint8[16] back as numpy; outgoing messages demand plain ints."""
    return [int(x) for x in t]


class Pad:
    """/dev/input/js0 via the Linux joystick API. 8-byte events: time,value,type,number."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes, self.buttons = {}, {}

    def poll(self):
        """Drain pending events. Returns the buttons that went down this tick."""
        pressed = []
        while True:
            try:
                buf = os.read(self.fd, 8)
            except BlockingIOError:
                return pressed
            if not buf or len(buf) < 8:
                return pressed
            _, value, etype, number = struct.unpack("IhBB", buf)
            etype &= ~0x80                       # strip the synthetic init flag
            if etype == 0x02:
                self.axes[number] = value / 32767.0
            elif etype == 0x01:
                was = self.buttons.get(number, 0)
                self.buttons[number] = value
                if value and not was:
                    pressed.append(number)

    def axis(self, n):
        v = self.axes.get(n, 0.0)
        return 0.0 if abs(v) < DEADZONE else v


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Teleop(Node):
    def __init__(self, args):
        super().__init__("teleop_xbox")
        self.args = args
        self.ee = None
        self.create_subscription(EeState, "/ee_state",
                                 lambda m: setattr(self, "ee", m), SENSOR_QOS)
        self.pub_pose = self.create_publisher(PoseSetpoint, "/setpoint/pose", SETPOINT_QOS)
        self.pub_twist = self.create_publisher(TwistSetpoint, "/setpoint/twist", SETPOINT_QOS)
        self.token = [0] * 16
        self.mode = "twist"
        self.target = None      # POSE mode integrator: (x, y, z, qx, qy, qz, qw)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def call(self, srv_type, name, req, timeout=8.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            sys.exit(f"service {name} never appeared -- is kinova_arm_node up?")
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        if not fut.done():
            sys.exit(f"service {name} timed out")
        return fut.result()

    # ---- session ------------------------------------------------------------
    def controller(self):
        return "ee_pose_position" if self.mode == "pose" else "ee_twist"

    def open(self):
        if self.args.dry_run:
            print(f"[dry-run] would open {self.controller()}")
            return
        # Publishers already exist and DDS has settled -- OpenStream.srv is explicit
        # that opening first sends the early setpoints nowhere.
        r = self.call(OpenStream, "open_stream", OpenStream.Request(
            controller=self.controller(), timeout_s=0.5, token=tok(self.token)))
        if not r.accepted:
            sys.exit(f"open_stream refused: {r.message}")
        print(f"streaming {self.controller()} on {list(r.channels)}")

    def close(self):
        if not self.args.dry_run:
            self.call(CloseStream, "close_stream",
                      CloseStream.Request(token=tok(self.token)))

    def toggle(self):
        self.close()
        self.mode = "pose" if self.mode == "twist" else "twist"
        self.target = None
        self.spin(0.3)                    # the 250 ms mode settle, plus a little
        self.open()

    # ---- the loop -----------------------------------------------------------
    def seed(self):
        p, o = self.ee.pose.position, self.ee.pose.orientation
        self.target = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def step(self, pad, dt):
        vx = -pad.axis(AX_LY) * self.args.max_lin      # stick up = +X away from base
        vy = -pad.axis(AX_LX) * self.args.max_lin
        vz = -pad.axis(AX_RY) * self.args.max_lin
        wz = -pad.axis(AX_RX) * self.args.max_ang

        if self.mode == "twist":
            m = TwistSetpoint(twist=Twist(), token=tok(self.token))
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = vx, vy, vz
            m.twist.angular.z = wz
            if self.args.dry_run:
                print(f"\rtwist  v=({vx:+.3f},{vy:+.3f},{vz:+.3f}) wz={wz:+.3f}", end="")
            else:
                self.pub_twist.publish(m)
            return

        if self.ee is None:
            return
        if self.target is None:
            self.seed()
        self.target[0] += vx * dt
        self.target[1] += vy * dt
        self.target[2] += vz * dt
        if wz:
            half = wz * dt / 2.0
            self.target[3:] = quat_mul((0.0, 0.0, math.sin(half), math.cos(half)),
                                       tuple(self.target[3:]))
        # The leash. An integrator with no bound can walk the target away from the arm
        # whenever IK cannot keep up; this makes that impossible by construction.
        meas = self.ee.pose.position
        d = [self.target[i] - c for i, c in enumerate((meas.x, meas.y, meas.z))]
        dist = math.sqrt(sum(v * v for v in d))
        if dist > LEASH_M:
            k = LEASH_M / dist
            for i, c in enumerate((meas.x, meas.y, meas.z)):
                self.target[i] = c + d[i] * k

        m = PoseSetpoint(pose=Pose(), token=tok(self.token))
        (m.pose.position.x, m.pose.position.y, m.pose.position.z,
         m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = self.target
        if self.args.dry_run:
            print(f"\rpose   p=({self.target[0]:+.3f},{self.target[1]:+.3f},"
                  f"{self.target[2]:+.3f}) leash={dist:.3f}m", end="")
        else:
            self.pub_pose.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--js", default="/dev/input/js0")
    ap.add_argument("--max-lin", type=float, default=0.05, help="m/s")
    ap.add_argument("--max-ang", type=float, default=0.3, help="rad/s")
    ap.add_argument("--mode", choices=("twist", "pose"), default="twist")
    ap.add_argument("--dry-run", action="store_true",
                    help="print setpoints; open no stream, command nothing")
    args = ap.parse_args()

    rclpy.init()
    n = Teleop(args)
    n.mode = args.mode
    pad = Pad(args.js)

    if not args.dry_run:
        r = n.call(AcquireControl, "acquire_control",
                   AcquireControl.Request(owner_id="teleop_xbox"))
        if not r.accepted:
            sys.exit(f"acquire_control refused: {r.message}")
        n.token = tok(r.token)
        print(f"owned, generation={r.generation}")

    n.spin(0.5)                            # let DDS discovery settle before opening
    n.open()
    print(f"mode={n.mode}  A=toggle  B=quit   lin={args.max_lin} m/s ang={args.max_ang} rad/s")

    dt = 1.0 / RATE_HZ
    try:
        while True:
            for b in pad.poll():
                if b == BTN_B:
                    return
                if b == BTN_A:
                    print()
                    n.toggle()
                    print(f"mode={n.mode}")
            n.step(pad, dt)
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nclosing stream, releasing control ...")
        try:
            n.close()
            if not args.dry_run:
                n.call(ReleaseControl, "release_control",
                       ReleaseControl.Request(token=tok(n.token)))
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
