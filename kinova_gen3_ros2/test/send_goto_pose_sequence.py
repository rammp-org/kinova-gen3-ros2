#!/usr/bin/env python3
"""Walk the EE through a sequence of GoToEEPose targets, one at a time.

Each pose is a base_link `tool_frame` target (position in metres, orientation as
an xyzw quaternion). cuRobo plans a collision-free trajectory from the live
/joint_states for each; we wait for it to finish before sending the next.

By default it samples 3 random waypoints in a workspace box in FRONT of the arm
(tool pointing down). cuRobo still plans/collision-checks each, so an unreachable
or colliding sample comes back PLANNING_FAILED rather than moving.

SAFETY: this executes at cuRobo's full planned speed. It is a DRY RUN by default
(prints the sequence and exits). Pass --go to actually move the arm — attended,
e-stop in hand, per docs/on-robot-runbook.md. Tune the WS_* box below for your
cell, or pass --poses FILE to use explicit targets instead of sampling.

Examples:
    python3 send_goto_pose_sequence.py                 # dry run: 3 random waypoints
    python3 send_goto_pose_sequence.py --go            # sample + execute 3
    python3 send_goto_pose_sequence.py -n 5 --seed 7 --go
    python3 send_goto_pose_sequence.py --poses seq.json --go
"""

import argparse
import json
import math
import random

# ROS is only needed to actually send goals (--go); a dry run works without it,
# so you can preview/edit the sequence on any box.
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from kinova_gen3_interfaces.action import GoToEEPose

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# Workspace box in base_link (metres) for random EE sampling — a conservative
# region in FRONT of the arm. Tune for your cell. Quat is xyzw; [1,0,0,0] points
# the tool down (tool_frame Z -> -Z base).
WS_X = (0.35, 0.55)  # forward from the base
WS_Y = (-0.25, 0.25)  # left / right
WS_Z = (0.20, 0.45)  # height above the base
TOOL_DOWN = [1.0, 0.0, 0.0, 0.0]


def sample_poses(n, seed):
    """n random tool targets uniform in the WS_* box, tool pointing down."""
    rng = random.Random(seed)
    return [
        {
            "name": f"rand_{i + 1}",
            "pos": [
                round(rng.uniform(*WS_X), 3),
                round(rng.uniform(*WS_Y), 3),
                round(rng.uniform(*WS_Z), 3),
            ],
            "quat": list(TOOL_DOWN),
        }
        for i in range(n)
    ]


# GoToEEPose result codes (from GoToEEPose.action).
_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
}


def _load_poses(path):
    with open(path) as f:
        poses = json.load(f)
    if not isinstance(poses, list) or not poses:
        raise ValueError("poses file must be a non-empty JSON list")
    return poses


def _normalize(quat):
    n = math.sqrt(sum(c * c for c in quat))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {quat}")
    return [c / n for c in quat]


def _send_one(node, client, pose, sender_id):
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

    def on_fb(fb):
        f = fb.feedback
        node.get_logger().info(
            f"  [{f.phase}] planner_state='{f.planner_state}' frac={f.fraction_complete:.2f}"
        )

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        node.get_logger().error("  goal REJECTED (frame must be base_link)")
        return -1
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    label = _CODES.get(res.error_code, str(res.error_code))
    node.get_logger().info(
        f"  result={label} ({res.error_code}) msg='{res.error_string}'"
    )
    return res.error_code


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "-n",
        "--count",
        type=int,
        default=3,
        help="number of random waypoints to sample (default 3)",
    )
    ap.add_argument(
        "--seed",
        type=int,
        default=None,
        help="RNG seed for reproducible sampling (default: fresh each run)",
    )
    ap.add_argument(
        "--poses",
        metavar="FILE",
        help="JSON list of {name,pos:[x,y,z],quat:[x,y,z,w]} — use instead of sampling",
    )
    ap.add_argument(
        "--go",
        action="store_true",
        help="actually move the arm (default: dry run — print and exit)",
    )
    ap.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="keep going after a pose that does not settle SUCCESSFUL",
    )
    ap.add_argument("--sender-id", default="send_goto_pose_sequence")
    args = ap.parse_args()

    if args.poses:
        poses = _load_poses(args.poses)
        src = f"file {args.poses}"
    else:
        seed = args.seed if args.seed is not None else random.randrange(1 << 30)
        poses = sample_poses(args.count, seed)
        src = f"random in front-of-arm box, seed={seed}"

    print(f"Sequence of {len(poses)} pose(s) (base_link, tool_frame; {src}):")
    for i, p in enumerate(poses):
        name = p.get("name", f"pose_{i + 1}")
        print(f"  {i + 1}. {name:>10}  pos={p['pos']}  quat(xyzw)={p['quat']}")
    if not args.go:
        print(
            "\nDry run — nothing sent. Re-run with --go to execute (attended, e-stop in hand)."
        )
        return 0

    if not _HAVE_ROS:
        print(
            "\n--go needs a ROS2 environment (rclpy + kinova_gen3_interfaces); none found."
        )
        return 1

    rclpy.init()
    node = Node("send_goto_pose_sequence")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        rclpy.shutdown()
        return 1

    failures = 0
    try:
        for i, pose in enumerate(poses):
            name = pose.get("name", f"pose_{i + 1}")
            node.get_logger().info(f"[{i + 1}/{len(poses)}] {name} -> {pose['pos']}")
            code = _send_one(node, client, pose, args.sender_id)
            if code != 0:
                failures += 1
                if not args.continue_on_fail:
                    node.get_logger().error(
                        f"stopping: {name} did not succeed (code {code})"
                    )
                    break
    finally:
        rclpy.shutdown()

    print(f"\nDone: {len(poses) - failures}/{len(poses)} pose(s) succeeded.")
    return 0 if failures == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
