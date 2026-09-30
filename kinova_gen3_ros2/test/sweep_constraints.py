#!/usr/bin/env python3
"""Run the SAME motion under every combination of constraint, and table the results.

This is the A/B rig: one fixed pose pair, executed once per case, so the effect of
each option is a difference in one row rather than something you have to take on
faith. Every case re-homes to the same START first (unconstrained, full speed), so
the timed leg always begins from the same place and the wall times compare.

What the table shows:

  speed_scale   The same leg at 1.0 and at 0.25 differs ONLY in wall time, ~4x.
                Same path, same waypoints — the executor dilates its clock.
  axis_lock     Held AT THE GOAL'S VALUE, so a lock is only satisfiable if START
                already matches TARGET on that component. The default pose pair
                moves in y and z at constant x and constant orientation, so:
                  roll/pitch/yaw, x  -> satisfiable, expect a move
                  y, z              -> NOT satisfiable, expect a REFUSAL naming
                                       the axis. That refusal is the pre-check
                                       doing its job, and the run asserts it.
  approach      Implies holding the other five components (see ApproachOffset.msg),
                so it is only satisfiable along an axis the motion actually travels.

Each case carries an EXPECTATION (move / refuse). The final table flags any case
whose outcome disagreed, which makes this a check and not just a demo.

ARBITRATION: like the other test clients here, this sends no capability
token. Under kEnforced every goal comes back refused and the table fills with
rejections that look like constraint failures but are not. Run it in permissive
mode, or acquire a token first.

SAFETY: DRY RUN by default — prints the full matrix and exits. Pass --go to
actually move, attended, e-stop in hand, per docs/on-robot-runbook.md. With --go
this executes 2 goals per case, so the default 8-case matrix is 16 moves. Tune
START/TARGET for your cell before the first --go.

Examples:
    python3 sweep_constraints.py                  # dry run: print the matrix
    python3 sweep_constraints.py --only speed     # just the speed_scale rows
    python3 sweep_constraints.py --list           # case names, one per line
    python3 sweep_constraints.py --go             # MOVES THE ARM (16 moves)
"""

import argparse
import math
import time

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rammp_arm_interfaces.action import GoToEEPose
    from rammp_arm_interfaces.msg import ApproachOffset, ToolAxisLock

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# The fixed pose pair, base_link metres, xyzw. Same x, same orientation; the
# motion is entirely in y and z. That is what makes lock_x and the orientation
# locks satisfiable while lock_y and lock_z are correctly refused.
TOOL_DOWN = [1.0, 0.0, 0.0, 0.0]
START = {"name": "start", "pos": [0.45, -0.25, 0.25], "quat": list(TOOL_DOWN)}
# The wide leg: travels in y AND z at constant x and constant orientation.
TARGET = {"name": "target", "pos": [0.45, 0.25, 0.50], "quat": list(TOOL_DOWN)}
# The descent leg: travels ONLY in z. An approach along z holds the other five
# components, and this is the pose pair where that hold is actually satisfiable —
# it is the "come down onto the object from above" move.
TARGET_Z = {"name": "target_z", "pos": [0.45, -0.25, 0.50], "quat": list(TOOL_DOWN)}
_TARGETS = {"wide": TARGET, "descent": TARGET_Z}

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
_FRAMES = {"base": 0, "goal": 1}

_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}

# group, name, target, speed_scale, locks, approach (distance, axis, fraction), expectation
#
# The expectation follows one rule: a held component must already match between
# START and the target. "wide" travels in y and z; "descent" travels only in z.
CASES = [
    ("speed", "baseline-full", "wide", 1.0, [], None, "move"),
    ("speed", "half", "wide", 0.5, [], None, "move"),
    ("speed", "quarter", "wide", 0.25, [], None, "move"),
    ("lock", "lock-roll-pitch", "wide", 1.0, ["roll", "pitch"], None, "move"),
    ("lock", "lock-yaw", "wide", 1.0, ["yaw"], None, "move"),
    ("lock", "lock-x-constant-axis", "wide", 1.0, ["x"], None, "move"),
    # y is travelled on this leg, so holding it cannot be satisfied from START.
    ("lock", "lock-y-travelled-axis", "wide", 1.0, ["y"], None, "refuse"),
    # An approach holds the OTHER FIVE components. On the descent leg only z
    # changes, so those five already match and the approach is satisfiable.
    ("approach", "approach-z-descent", "descent", 1.0, [], (0.10, "z", 0.8), "move"),
    # Same approach on the wide leg: y is held at the goal value but START's y
    # differs, so the pre-check refuses it and names the axis.
    ("approach", "approach-z-wide-leg", "wide", 1.0, [], (0.10, "z", 0.8), "refuse"),
    # Slow descent with the approach — the pairing a real grasp actually uses.
    ("approach", "approach-z-slow", "descent", 0.25, [], (0.10, "z", 0.8), "move"),
]


def _normalize(quat):
    n = math.sqrt(sum(c * c for c in quat))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {quat}")
    return [c / n for c in quat]


def _build_goal(pose, sender_id, speed=1.0, locks=(), approach=None, frame="base"):
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
    goal.speed_scale = speed
    goal.axis_lock = ToolAxisLock()
    goal.axis_lock.reference_frame = _FRAMES[frame]
    for name in locks:
        setattr(goal.axis_lock, f"lock_{name}", True)
    goal.approach_offset = ApproachOffset()
    if approach:
        distance, axis, at_fraction = approach
        goal.approach_offset.distance = distance
        goal.approach_offset.axis = _AXIS_INDEX[axis]
        goal.approach_offset.at_fraction = at_fraction
    return goal


def _send(node, client, goal, quiet=False):
    """Send one goal; return (error_code, wall_seconds). A rejected goal is -1.

    Rejections carry no payload (kinova-gen3-ros2 #39), so a refusal's reason is
    only in the node's log — the code is all that reaches us here.
    """
    t0 = time.monotonic()

    def on_fb(fb):
        if not quiet:
            f = fb.feedback
            node.get_logger().info(f"    [{f.phase}] frac={f.fraction_complete:.2f}")

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        return -1, time.monotonic() - t0
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    return res.error_code, time.monotonic() - t0


def describe(case):
    _group, name, target, speed, locks, approach, expect = case
    lock_s = "+".join(locks) if locks else "-"
    app_s = f"{approach[0]}m/{approach[1]}@{approach[2]}" if approach else "-"
    return name, target, speed, lock_s, app_s, expect


def print_matrix(cases):
    print(f"\n  START    {START['pos']}")
    print(f"  wide     {TARGET['pos']}   travels in y and z")
    print(f"  descent  {TARGET_Z['pos']}   travels in z only")
    print("  x and tool orientation are constant everywhere\n")
    head = (
        f"  {'case':<22} {'leg':<8} {'speed':>6}  {'locks':<14} "
        f"{'approach':<14} {'expect':<7}"
    )
    print(head)
    print("  " + "-" * (len(head) - 2))
    for c in cases:
        name, target, speed, lock_s, app_s, expect = describe(c)
        print(
            f"  {name:<22} {target:<8} {speed:>6}  {lock_s:<14} "
            f"{app_s:<14} {expect:<7}"
        )
    print()


def print_results(rows):
    print("\n  ==== results ====\n")
    head = (
        f"  {'case':<24} {'expect':<7} {'got':<18} {'wall':>7}  {'ok':<3}"
    )
    print(head)
    print("  " + "-" * (len(head) - 2))
    bad = 0
    for name, expect, code, wall in rows:
        got = _CODES.get(code, str(code))
        moved = code == 0
        agreed = moved == (expect == "move")
        if not agreed:
            bad += 1
        print(
            f"  {name:<24} {expect:<7} {got:<18} {wall:>6.2f}s  {'yes' if agreed else 'NO':<3}"
        )
    print()
    if bad:
        print(f"  {bad} case(s) disagreed with their expectation\n")
    else:
        print("  every case matched its expectation\n")
    return bad


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--only",
        choices=sorted({c[0] for c in CASES}),
        help="run just one group of cases (speed / lock / approach)",
    )
    ap.add_argument("--list", action="store_true", help="print case names and exit")
    ap.add_argument("--frame", choices=sorted(_FRAMES), default="base")
    ap.add_argument("--sender-id", default="sweep_constraints")
    ap.add_argument(
        "--go", action="store_true", help="ACTUALLY MOVE THE ARM. Without this, dry run."
    )
    args = ap.parse_args()

    cases = [c for c in CASES if not args.only or c[0] == args.only]
    if args.list:
        for c in cases:
            print(c[1])
        return 0

    print_matrix(cases)

    if not args.go:
        print(
            f"  DRY RUN — nothing was sent. {len(cases)} cases x 2 goals = "
            f"{len(cases) * 2} moves when you pass --go.\n"
        )
        return 0

    if not _HAVE_ROS:
        print("  --go needs rclpy + rammp_arm_interfaces on the path.\n")
        return 2

    rclpy.init()
    node = Node("sweep_constraints")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        rclpy.shutdown()
        return 1

    rows = []
    for _group, name, target_key, speed, locks, approach, expect in cases:
        node.get_logger().info(f"=== {name} ({target_key} leg)")
        # Re-home unconstrained and at full speed so the timed leg below always
        # starts from the same place. A failure here invalidates the case.
        home_code, _ = _send(
            node, client, _build_goal(START, args.sender_id, frame=args.frame), quiet=True
        )
        if home_code != 0:
            node.get_logger().error(
                f"  could not re-home before '{name}' (code={home_code}); skipping it"
            )
            rows.append((name, expect, home_code, float("nan")))
            continue
        goal = _build_goal(
            _TARGETS[target_key], args.sender_id, speed, locks, approach, args.frame
        )
        code, wall = _send(node, client, goal)
        node.get_logger().info(f"  {_CODES.get(code, code)} in {wall:.2f}s")
        rows.append((name, expect, code, wall))

    rclpy.shutdown()
    bad = print_results(rows)
    return 0 if bad == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
