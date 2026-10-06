#!/usr/bin/env python3
"""Run a tour of joint-space waypoints through GoToJointConfig, logging /joint_states.

Each leg is one go_to_joint_config goal: the driver asks cuRobo for a plan from
the live state and executes it -- the planned-trajectory path whose smoothness
depends on the driver's sampler rate. Run the same tour against the 250 Hz and
1 kHz builds and compare the printed velocity ripple (or the CSVs).

The ripple number is the RMS of measured joint velocity minus its own moving
average, while the arm is moving. /joint_states is ~100 Hz, so a 250 Hz
staircase shows up aliased rather than resolved: compare runs with it, don't
read it as an absolute.

SAFETY: this executes at cuRobo's full planned speed. DRY RUN by default; pass
--go to move the arm -- attended, e-stop in hand, per docs/on-robot-runbook.md.

  send_joint_tour.py                                   # dry run: print the tour
  send_joint_tour.py --go --loops 3 --csv tour_1khz.csv
  send_joint_tour.py --waypoints tour.json --go        # JSON list of 7-joint lists, rad
  send_joint_tour.py --go --mode impedance --csv tour_imp.csv   # compliant execution
  send_joint_tour.py --go --mode impedance --profile soft       # same, soft gains

With --mode impedance each leg also prints the result's final joint-space
error (max |rad| across joints) -- the spec §5 "track" number.
"""

import argparse
import csv
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rammp_arm_interfaces.action import GoToJointConfig

from gains_cli import add_gains_args, build_gains

RESULT_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}

# The node's default `home` preset (cuRobo's retract configuration).
HOME = [0.0, 0.262, 3.142, -2.269, 0.0, 0.960, 1.571]
# Default tour as offsets from HOME, rad. Kept well inside the bounded joints'
# limits (joint_2 +/-2.41, joint_4 +/-2.66, joint_6 +/-2.23); cuRobo still
# collision-checks every leg.
OFFSETS = [
    [0.5, 0.25, 0.0, 0.35, 0.0, -0.25, 0.0],
    [-0.5, 0.25, 0.0, 0.35, 0.0, -0.25, 0.0],
    [0.0, -0.15, 0.0, 0.0, 0.5, 0.30, 0.6],
    [0.0] * 7,
]


def default_tour():
    return [[h + d for h, d in zip(HOME, off)] for off in OFFSETS]


def velocity_ripple(qd, window=5, moving_rad_s=0.05):
    """RMS of qd minus its centred moving average, over samples where the arm moves."""
    qd = np.asarray(qd, dtype=float)
    if len(qd) < 3 * window:
        return float("nan")
    k = np.ones(window) / window
    smooth = np.column_stack(
        [np.convolve(qd[:, j], k, mode="same") for j in range(qd.shape[1])]
    )
    resid, smooth = (qd - smooth)[window:-window], smooth[window:-window]
    moving = np.abs(smooth).max(axis=1) > moving_rad_s
    if not moving.any():
        return float("nan")
    return float(np.sqrt(np.mean(resid[moving] ** 2)))


class Tour(Node):
    def __init__(self):
        super().__init__("send_joint_tour")
        self.leg = -1
        self.samples = []  # (t, leg, q[7], qd[7])
        # /joint_states is BEST_EFFORT; a RELIABLE subscription receives nothing.
        self.create_subscription(
            JointState, "/joint_states", self._on_js, qos_profile_sensor_data
        )
        self.client = ActionClient(self, GoToJointConfig, "go_to_joint_config")

    def _on_js(self, msg):
        if len(msg.position) >= 7 and len(msg.velocity) >= 7:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append(
                (t, self.leg, list(msg.position[:7]), list(msg.velocity[:7]))
            )

    def go_to(self, target, sender_id, control_mode, gains, speed_scale):
        goal = GoToJointConfig.Goal()
        goal.target_joints = [float(v) for v in target]
        goal.sender_id = sender_id
        goal.control_mode = control_mode
        goal.gains = gains
        goal.speed_scale = speed_scale
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if gh is None or not gh.accepted:
            return -1, "goal rejected by the server", None
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        r = res.result().result
        final_err = (
            max(abs(e) for e in r.final_error.positions)
            if r.final_error.positions
            else None
        )
        return r.error_code, r.error_string, final_err


def write_csv(path, samples):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["t", "leg"]
            + [f"q{i + 1}" for i in range(7)]
            + [f"qd{i + 1}" for i in range(7)]
        )
        for t, leg, q, qd in samples:
            w.writerow([f"{t:.6f}", leg] + q + qd)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--waypoints",
        metavar="FILE",
        help="JSON list of 7-joint lists (rad); default: a small tour around home",
    )
    ap.add_argument("--loops", type=int, default=1, help="times to run the tour")
    ap.add_argument(
        "--csv", metavar="PATH", help="write every /joint_states sample here"
    )
    ap.add_argument(
        "--go", action="store_true", help="actually move the arm (default: dry run)"
    )
    ap.add_argument("--sender-id", default="send_joint_tour")
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="execution pace; 1.0 = as planned. Refused outside [0.01, 1.0].",
    )
    ap.add_argument(
        "--mode",
        choices=["position", "impedance"],
        default="position",
        help="execute each leg stiff (position) or compliant (impedance)",
    )
    add_gains_args(ap)
    args = ap.parse_args()
    # Same bounds the node enforces, refused rather than clamped -- catching it
    # here saves a round trip and gives a reason, which a rejection cannot carry.
    if not 0.01 <= args.speed_scale <= 1.0:
        ap.error(f"--speed-scale must be in [0.01, 1.0]; got {args.speed_scale}")
    control_mode = (
        GoToJointConfig.Goal.CONTROL_MODE_IMPEDANCE
        if args.mode == "impedance"
        else GoToJointConfig.Goal.CONTROL_MODE_POSITION
    )
    gains = build_gains(args)

    if args.waypoints:
        with open(args.waypoints) as f:
            waypoints = json.load(f)
    else:
        waypoints = default_tour()
    if not waypoints or any(len(w) != 7 for w in waypoints):
        print("waypoints must be a non-empty list of 7-joint lists")
        return 2
    tour = waypoints * args.loops

    print(
        f"Tour: {len(waypoints)} waypoint(s) x {args.loops} loop(s), "
        f"{args.mode} (gains: "
        f"{'custom' if gains.profile == gains.PROFILE_CUSTOM else args.profile}, "
        f"speed {args.speed_scale:g})"
    )
    for i, w in enumerate(waypoints):
        print(f"  {i + 1}. [" + ", ".join(f"{v:+.3f}" for v in w) + "]")
    if not args.go:
        print(
            "\nDry run -- nothing sent. Re-run with --go to execute (attended, e-stop in hand)."
        )
        return 0

    rclpy.init()
    node = Tour()
    try:
        if not node.client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("go_to_joint_config action server unavailable")
            return 1
        done, t0 = 0, time.monotonic()
        for i, target in enumerate(tour):
            node.leg = i
            start = time.monotonic()
            code, msg, final_err = node.go_to(
                target, args.sender_id, control_mode, gains, args.speed_scale
            )
            label = RESULT_CODES.get(code, str(code))
            err_txt = f" final_err={final_err:.4f} rad" if final_err is not None else ""
            print(
                f"[{i + 1}/{len(tour)}] {label} in {time.monotonic() - start:.2f} s{err_txt} {msg}"
            )
            if code != 0:
                break
            done += 1
        node.leg = -1

        ripple = velocity_ripple([s[3] for s in node.samples])
        print(
            f"\n{done}/{len(tour)} legs succeeded in {time.monotonic() - t0:.1f} s; "
            f"{len(node.samples)} /joint_states samples; "
            f"velocity ripple RMS = {ripple:.4f} rad/s"
        )
        if args.csv:
            write_csv(args.csv, node.samples)
            print(f"wrote {args.csv}")
        return 0 if done == len(tour) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
