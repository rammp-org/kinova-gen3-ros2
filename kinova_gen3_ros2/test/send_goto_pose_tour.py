#!/usr/bin/env python3
"""Walk the EE around a large, deterministic tour — the demo lap for speed_scale,
orientation holds.

Where send_goto_pose_sequence.py samples RANDOM poses in a small conservative box
(a reachability probe), this walks a fixed RING of widely-spaced waypoints in front
of the arm. The motions are deliberately big, so a change in speed_scale or a held
axis is visible from across the room rather than needing a plot.

The three things it shows off:

  speed_scale   Per-leg pace. The path is identical; only the clock changes. Run
                the same lap at 1.0 and at 0.25 and it takes ~4x as long.
  hold          Keep the tool's orientation for the WHOLE leg, held AT THE
                GOAL'S VALUE — see OrientationHold.msg. Every ring waypoint
                shares one orientation, so a hold is satisfiable on every leg;
                `level` through a whole lap is the "carry a full cup around the
                room" demo.

ARBITRATION: like the other test clients here, this sends no capability
token. Under kEnforced every goal comes back refused and the table fills with
rejections that look like constraint failures but are not. Run it in permissive
mode, or acquire a token first.

SAFETY: these are LARGE motions at cuRobo's planned speed. DRY RUN by default —
it prints the tour, the leg lengths and the per-leg settings, and exits. Pass --go
to actually move, attended, e-stop in hand, per docs/on-robot-runbook.md. Tune the
RING_* constants for your cell before the first --go.

Examples:
    python3 send_goto_pose_tour.py                      # dry run: print the lap
    python3 send_goto_pose_tour.py --speed-scale 0.25   # dry run of a slow lap
    python3 send_goto_pose_tour.py --hold level         # dry run, tool held flat
    python3 send_goto_pose_tour.py --dump tour.json     # save it for reuse
    python3 send_goto_pose_tour.py --go                 # MOVES THE ARM
"""

import argparse
import json
import math

# ROS is only needed to actually send goals (--go); a dry run works without it,
# so the lap can be previewed and edited on any box.
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rammp_arm_interfaces.action import GoToEEPose
    from rammp_arm_interfaces.msg import OrientationHold

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# The ring, in base_link metres. A tour sweeps an arc in FRONT of the arm at two
# heights, so each leg moves the tool a long way in both y and z. Conservative
# enough to stay inside the Gen3's comfortable envelope, big enough to see.
RING_RADIUS = 0.50  # forward reach of the arc
RING_Y_SPAN = 0.70  # total left-to-right sweep
RING_Z_LOW = 0.25  # height of the low pass
RING_Z_HIGH = 0.55  # height of the high pass
# Gripper level, approach axis horizontal facing forward. READ OFF THE ARM, not
# derived: an earlier value put tool Z forward but rolled the gripper 90 deg
# about it, which cuRobo could solve for 3 of 25 probed poses.
GRIPPER_LEVEL = [0.5, 0.5, 0.5, 0.5]

# Values match OrientationHold's constants; spelled out so a dry run needs no ROS.
_HOLDS = {"none": 0, "level": 1, "fixed": 2}


# GoToEEPose result codes (from GoToEEPose.action).
_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}


def build_tour(n):
    """n waypoints around the ring, alternating low and high so every leg is a
    big diagonal rather than a slide along one axis.

    Every waypoint shares ONE orientation, which is what makes a hold
    satisfiable on every leg: a hold keeps the orientation at the goal's value,
    so the previous waypoint already matches it. An earlier version appended a
    dedicated descent leg to carry an approach offset; that field is gone
    (kinova-gen3-ros2#40) and the ring needs no special case.
    """
    if n < 2:
        raise ValueError("a tour needs at least 2 waypoints")
    out = []
    for i in range(n):
        # Sweep y across the span; x follows the arc so the tool stays at a
        # roughly constant reach instead of stretching at the edges.
        frac = i / (n - 1)
        y = -RING_Y_SPAN / 2.0 + frac * RING_Y_SPAN
        x = math.sqrt(max(RING_RADIUS**2 - y**2, 0.04))
        z = RING_Z_HIGH if i % 2 else RING_Z_LOW
        out.append(
            {
                "name": f"wp{i + 1}",
                "pos": [round(x, 3), round(y, 3), round(z, 3)],
                "quat": list(GRIPPER_LEVEL),
            }
        )
    return out


def leg_length(a, b):
    return math.dist(a["pos"], b["pos"])


def _normalize(quat):
    n = math.sqrt(sum(c * c for c in quat))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {quat}")
    return [c / n for c in quat]


def _send_one(node, client, pose, args, sender_id):
    """Send one GoToEEPose goal; return its integer error_code (or a synthetic
    negative for reject/no-result). Blocks until the goal settles."""
    goal = GoToEEPose.Goal()
    goal.target.header.frame_id = "base_link"
    (
        goal.target.pose.position.x,
        goal.target.pose.position.y,
        goal.target.pose.position.z,
    ) = pose["pos"]
    (
        goal.target.pose.orientation.x,
        goal.target.pose.orientation.y,
        goal.target.pose.orientation.z,
        goal.target.pose.orientation.w,
    ) = _normalize(pose["quat"])
    goal.sender_id = sender_id
    goal.speed_scale = args.speed_scale

    goal.orientation_hold = OrientationHold()
    goal.orientation_hold.hold = _HOLDS[args.hold]


    def on_fb(fb):
        f = fb.feedback
        node.get_logger().info(
            f"  [{f.phase}] planner_state='{f.planner_state}' frac={f.fraction_complete:.2f}"
        )

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        # Rejections carry no payload (see kinova-gen3-ros2 #39), so the reason
        # is only in the node's log — check there, not here.
        node.get_logger().error("  goal REJECTED — reason is in the node log")
        return -1
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    label = _CODES.get(res.error_code, str(res.error_code))
    node.get_logger().info(
        f"  result={label} ({res.error_code}) msg='{res.error_string}'"
    )
    return res.error_code


def print_tour(tour, args):
    total = sum(leg_length(tour[i], tour[i + 1]) for i in range(len(tour) - 1))
    print(f"\n  tour: {len(tour)} waypoints, {total:.2f} m of travel")
    print(f"  speed_scale  {args.speed_scale}")
    print(f"  hold         {args.hold}")
    print()
    for i, p in enumerate(tour):
        x, y, z = p["pos"]
        step = f"  +{leg_length(tour[i - 1], p):.2f} m" if i else ""
        print(f"    {p['name']:>5}  x={x:+.3f} y={y:+.3f} z={z:+.3f}{step}")
    print()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("-n", "--count", type=int, default=5, help="waypoints in the lap")
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="pace for every leg; 1.0 = as planned. Refused outside [0.01, 1.0].",
    )
    ap.add_argument(
        "--hold",
        choices=sorted(_HOLDS),
        default="none",
        help="keep the tool's orientation for every leg: none, level (tilt "
        "held, yaw free) or fixed (all three held)",
    )
    ap.add_argument("--dump", metavar="FILE", help="write the tour as JSON and exit")
    ap.add_argument("--sender-id", default="send_goto_pose_tour")
    ap.add_argument(
        "--go",
        action="store_true",
        help="ACTUALLY MOVE THE ARM. Without this it is a dry run.",
    )
    args = ap.parse_args()

    # Validate locally so a dry run catches a bad value too — the same bounds the
    # node enforces, refused rather than clamped.
    if not 0.01 <= args.speed_scale <= 1.0:
        ap.error(f"--speed-scale must be in [0.01, 1.0]; got {args.speed_scale}")


    tour = build_tour(args.count)
    print_tour(tour, args)

    if args.dump:
        with open(args.dump, "w") as f:
            json.dump(tour, f, indent=2)
        print(f"  wrote {args.dump}\n")
        return 0

    if not args.go:
        print("  DRY RUN — nothing was sent. Pass --go to move the arm.\n")
        return 0

    if not _HAVE_ROS:
        print("  --go needs rclpy + rammp_arm_interfaces on the path.\n")
        return 2

    rclpy.init()
    node = Node("send_goto_pose_tour")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        rclpy.shutdown()
        return 1

    failed = 0
    for p in tour:
        node.get_logger().info(f"--> {p['name']} {p['pos']}")
        if _send_one(node, client, p, args, args.sender_id) != 0:
            failed += 1
            node.get_logger().error(f"{p['name']} did not succeed — stopping the tour")
            break

    rclpy.shutdown()
    return 0 if failed == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
