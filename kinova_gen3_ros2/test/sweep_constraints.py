#!/usr/bin/env python3
"""Step through every orientation hold mode on ONE motion and table the results.

The A/B rig: one fixed pose pair, executed once per case, so the effect of each
setting is a difference in one row rather than something to take on faith. Every
case re-homes to the same START first, unconstrained and at full speed, so the
timed leg always begins from the same place and the wall times compare.

This was a ten-case matrix over six per-axis locks and an approach offset. Both
are gone -- per-axis holds expressed a mechanism rather than a request, and
position holds could only ever mean "travel along one base axis". What is left
is the whole interface: three modes and a speed.

  none / level / fixed    the three modes, same motion, full speed
  level at half speed     same path, same hold, about twice the wall time
  conflicting goal        a goal orientation that disagrees with the start on
                          the held components. REFUSED, not re-aimed: a hold
                          keeps the orientation at the GOAL's value, so that
                          request asks for two orientations at once.

Each case carries an EXPECTATION and the table flags any disagreement, so this
is a check rather than a demo.

ARBITRATION: like the other clients here this sends no capability token. Under
kEnforced every goal comes back refused and the table fills with rejections that
look like constraint failures but are not. Run permissive, or acquire a token.

SAFETY: DRY RUN by default -- prints the matrix and exits. Pass --go to move the
arm, attended, e-stop in hand. Each case is 2 goals, so the 5-case run is 10
moves. Tune START/TARGET for your cell before the first --go.

Examples:
    python3 sweep_constraints.py                  # dry run: print the matrix
    python3 sweep_constraints.py --list           # case names, one per line
    python3 sweep_constraints.py --go             # MOVES THE ARM, Enter per case
    python3 sweep_constraints.py --go --no-pause  # MOVES THE ARM, no prompts
"""

import argparse
import math
import time

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rammp_arm_interfaces.action import GoToEEPose
    from rammp_arm_interfaces.msg import OrientationHold

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# Gripper level, approach axis horizontal facing forward. READ OFF THE ARM, not
# derived -- see scenario_carry_level.py for why that distinction mattered.
GRIPPER_LEVEL = [0.5, 0.5, 0.5, 0.5]
# A rolled variant used ONLY by the conflicting-goal case. MEASURED, not
# guessed: 7.86 deg of pure tilt with 0.00 deg of yaw. Both halves matter --
# past the planner's 2-degree default so it is certainly refused, and zero yaw
# so the refusal can only be about the component LEVEL actually holds.
TILTED = [0.5327, 0.4642, 0.5327, 0.4642]

START = {"name": "start", "pos": [0.50, -0.20, 0.55], "quat": list(GRIPPER_LEVEL)}
TARGET = {"name": "target", "pos": [0.55, 0.20, 0.40], "quat": list(GRIPPER_LEVEL)}
# Same POSITION as TARGET so the only thing that can trip a hold is the
# orientation disagreement, not reachability.
TARGET_TILTED = {"name": "tilted", "pos": list(TARGET["pos"]), "quat": list(TILTED)}
_TARGETS = {"target": TARGET, "tilted": TARGET_TILTED}

_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}

# Numeric values match the message constants; spelled out so a DRY RUN works
# with no ROS on the path at all.
_HOLDS = {"none": 0, "level": 1, "fixed": 2}

# name, hold, speed, target key, expectation
CASES = [
    ("free", "none", 1.0, "target", "move"),
    ("level", "level", 1.0, "target", "move"),
    ("fixed", "fixed", 1.0, "target", "move"),
    ("level-half-speed", "level", 0.5, "target", "move"),
    ("level-conflicting-goal", "level", 1.0, "tilted", "refuse"),
]


def _normalize(quat):
    n = math.sqrt(sum(c * c for c in quat))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {quat}")
    return [c / n for c in quat]


def what_to_watch(case, baselines):
    _name, hold, speed, target, expect = case
    if expect == "refuse":
        return "nothing should move — the goal's orientation disagrees with the start"
    ref = baselines.get((hold, target))
    if ref and ref[1] != speed:
        return (
            f"the same path as '{hold}' at {ref[1]}, taking about "
            f"{ref[1] / speed:.0f}x as long"
        )
    if hold == "none":
        return "the planner reorients freely — this is the reference time"
    if hold == "level":
        return "watch the tool keep its tilt while yaw is free to change"
    return "watch the tool hold its orientation exactly — no reorientation at all"


def print_matrix(cases):
    print(f"\n  START   {START['pos']}  gripper level")
    print(f"  target  {TARGET['pos']}  same orientation as START")
    print(f"  tilted  {TARGET_TILTED['pos']}  same position, 7.9 deg of tilt\n")
    head = f"  {'case':<24} {'hold':<7} {'speed':>6}  {'goal':<8} {'expect':<7}"
    print(head)
    print("  " + "-" * (len(head) - 2))
    for name, hold, speed, target, expect in cases:
        print(f"  {name:<24} {hold:<7} {speed:>6}  {target:<8} {expect:<7}")
    print()


def print_case_banner(i, total, case, baselines):
    name, hold, speed, target, expect = case
    print("\n" + "  " + "─" * 68)
    print(f"  CASE {i}/{total}   {name}")
    print(f"    hold       {hold}")
    print(f"    speed      {speed}")
    print(f"    goal       {target}  {_TARGETS[target]['pos']}")
    print(f"    expect     {expect.upper()}")
    print(f"    watch      {what_to_watch(case, baselines)}")
    ref = baselines.get((hold, target))
    if ref and ref[1] != speed:
        print(
            f"    predict    ~{ref[0] * ref[1] / speed:.1f}s "
            f"(same case at {ref[1]} took {ref[0]:.2f}s)"
        )


def report_case(case, code, wall, baselines):
    _name, hold, speed, target, expect = case
    agreed = (code == 0) == (expect == "move")
    print(
        f"\n    result     {_CODES.get(code, str(code))} in {wall:.2f}s  —  "
        f"{'AS EXPECTED' if agreed else '*** DISAGREES WITH EXPECTATION ***'}"
    )
    ref = baselines.get((hold, target))
    if code == 0 and ref and ref[1] != speed:
        print(
            f"    compare    {wall:.2f}s / {ref[0]:.2f}s = {wall / ref[0]:.2f}x, "
            f"ideal {ref[1] / speed:.2f}x"
        )
        print(
            "               below ideal is expected: planning and the action "
            "round-trip are fixed cost and do not scale."
        )
    elif not agreed and expect == "refuse":
        print(
            "               it MOVED when the hold should have been "
            "unsatisfiable — the expectation may be wrong, not the planner."
        )
    return agreed


def print_results(rows):
    print("\n  ==== results ====\n")
    head = f"  {'case':<24} {'expect':<7} {'got':<18} {'wall':>7}  {'ok':<12}"
    print(head)
    print("  " + "-" * (len(head) - 2))
    bad = incon = 0
    for name, expect, code, wall, status in rows:
        if status == "inconclusive":
            # NOT a pass and NOT a failure: the setup broke, so the case never
            # ran. Scoring it would let a broken re-home masquerade as a
            # passing refusal.
            incon += 1
            print(
                f"  {name:<24} {expect:<7} {'(re-home failed)':<18} {'--':>7}  "
                f"{'INCONCLUSIVE':<12}"
            )
            continue
        agreed = (code == 0) == (expect == "move")
        bad += 0 if agreed else 1
        print(
            f"  {name:<24} {expect:<7} {_CODES.get(code, code):<18} "
            f"{wall:>6.2f}s  {'yes' if agreed else 'NO':<12}"
        )
    print()
    if incon:
        print(
            f"  {incon} case(s) INCONCLUSIVE — the re-home failed, so they never\n"
            "  ran. Fix that before reading anything into the rest.\n"
        )
    if bad:
        print(f"  {bad} case(s) disagreed with their expectation\n")
    elif not incon:
        print("  every case matched its expectation\n")
    return bad, incon


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--list", action="store_true", help="print case names and exit")
    ap.add_argument("--sender-id", default="sweep_constraints")
    ap.add_argument("--no-pause", action="store_true", help="no Enter between cases")
    ap.add_argument("--go", action="store_true", help="ACTUALLY MOVE THE ARM")
    args = ap.parse_args()

    if args.list:
        for c in CASES:
            print(c[0])
        return 0

    print_matrix(CASES)
    if not args.go:
        print(
            f"  DRY RUN — nothing was sent. {len(CASES)} cases x 2 goals = "
            f"{len(CASES) * 2} moves when you pass --go.\n"
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

    def goal_for(pose, hold="none", speed=1.0):
        g = GoToEEPose.Goal()
        g.target.header.frame_id = "base_link"
        (
            g.target.pose.position.x,
            g.target.pose.position.y,
            g.target.pose.position.z,
        ) = pose["pos"]
        (
            g.target.pose.orientation.x,
            g.target.pose.orientation.y,
            g.target.pose.orientation.z,
            g.target.pose.orientation.w,
        ) = _normalize(pose["quat"])
        g.sender_id = args.sender_id
        g.speed_scale = speed
        g.orientation_hold = OrientationHold()
        g.orientation_hold.hold = _HOLDS[hold]
        return g

    def send(goal):
        t0 = time.monotonic()
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            # Rejections carry no payload (kinova-gen3-ros2#39): the reason is
            # only in the node's log, never here.
            return -1, time.monotonic() - t0
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf)
        return rf.result().result.error_code, time.monotonic() - t0

    baselines, rows = {}, []
    try:
        for i, case in enumerate(CASES, 1):
            name, hold, speed, target, expect = case
            print_case_banner(i, len(CASES), case, baselines)
            if not args.no_pause:
                try:
                    ans = input("\n    [Enter] run   [s] skip   [q] quit: ")
                    ans = ans.strip().lower()
                except EOFError:  # piped stdin: just run
                    ans = ""
                if ans == "q":
                    print("\n  stopped at the operator's request.\n")
                    break
                if ans == "s":
                    rows.append((name, expect, None, float("nan"), "inconclusive"))
                    print("    skipped.")
                    continue

            print("    re-homing...", end=" ", flush=True)
            code, wall = send(goal_for(START))
            if code != 0:
                print(f"FAILED (code={code}) — case NOT run, recorded INCONCLUSIVE")
                rows.append((name, expect, code, float("nan"), "inconclusive"))
                continue
            print(f"ok ({wall:.2f}s)")

            print("    running...", end=" ", flush=True)
            code, wall = send(goal_for(_TARGETS[target], hold, speed))
            print("done")
            report_case(case, code, wall, baselines)
            # Baseline keyed by the whole configuration, so a ratio is only
            # ever taken against a run that differs in SPEED alone.
            if code == 0 and speed == 1.0:
                baselines.setdefault((hold, target), (wall, speed))
            rows.append((name, expect, code, wall, "ran"))
    finally:
        rclpy.shutdown()

    bad, incon = print_results(rows)
    return 0 if (bad == 0 and incon == 0) else 3


if __name__ == "__main__":
    raise SystemExit(main())
