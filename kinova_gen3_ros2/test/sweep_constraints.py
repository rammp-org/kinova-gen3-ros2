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

With --go it STEPS: before each case it prints what is about to happen and what
you should see, and waits for Enter (or `s` to skip, `q` to stop). After the case
it prints the outcome and, for a speed case, the measured ratio against
baseline-full. Pass --no-pause for an unattended run.

Examples:
    python3 sweep_constraints.py                  # dry run: print the matrix
    python3 sweep_constraints.py --only speed     # just the speed_scale rows
    python3 sweep_constraints.py --list           # case names, one per line
    python3 sweep_constraints.py --go             # MOVES THE ARM, one Enter at a time
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
# The z-only leg: travels ONLY in z (upward here, 0.25 -> 0.50). An approach
# along z holds the other five components, and this is the pose pair where that
# hold is actually satisfiable. NOTE it rises rather than descends; the constraint
# maths is direction-agnostic, but if you want the realistic "come down onto the
# object" demo, lower TARGET_Z below START instead (the table top is at z=-0.07,
# so there is room).
TARGET_Z = {"name": "target_z", "pos": [0.45, -0.25, 0.50], "quat": list(TOOL_DOWN)}
_TARGETS = {"wide": TARGET, "z_only": TARGET_Z}

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
# START and the target. "wide" travels in y and z; "z_only" travels only in z.
CASES = [
    ("speed", "baseline-full", "wide", 1.0, [], None, "move"),
    ("speed", "half", "wide", 0.5, [], None, "move"),
    ("speed", "quarter", "wide", 0.25, [], None, "move"),
    ("lock", "lock-roll-pitch", "wide", 1.0, ["roll", "pitch"], None, "move"),
    ("lock", "lock-yaw", "wide", 1.0, ["yaw"], None, "move"),
    ("lock", "lock-x-constant-axis", "wide", 1.0, ["x"], None, "move"),
    # y is travelled on this leg, so holding it cannot be satisfied from START.
    ("lock", "lock-y-travelled-axis", "wide", 1.0, ["y"], None, "refuse"),
    # An approach holds the OTHER FIVE components. On the z-only leg only z
    # changes, so those five already match and the approach is satisfiable.
    ("approach", "approach-z-zonly", "z_only", 1.0, [], (0.10, "z", 0.8), "move"),
    # Same approach on the wide leg: y is held at the goal value but START's y
    # differs, so the pre-check refuses it and names the axis.
    ("approach", "approach-z-wide-leg", "wide", 1.0, [], (0.10, "z", 0.8), "refuse"),
    # Slow z-only move with the approach — the pairing a real grasp uses.
    ("approach", "approach-z-slow", "z_only", 0.25, [], (0.10, "z", 0.8), "move"),
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
    print(f"  z_only   {TARGET_Z['pos']}   travels in z only")
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


_ROTATIONAL = {"roll", "pitch", "yaw"}


def leg_shape(target_key):
    """Describe the motion from the actual coordinates, not from the leg's name."""
    tgt = _TARGETS[target_key]
    dz = tgt["pos"][2] - START["pos"][2]
    if target_key == "wide":
        return "a long diagonal in y and z"
    return f"straight {'up' if dz > 0 else 'down'} in z"


def baseline_for(case, baselines):
    """The full-speed reference for THIS case's leg, or None.

    A wall time is only comparable against a run that differs ONLY in speed, so
    the key is the whole configuration -- leg, locks and approach. Dividing a
    z-only time by the wide leg's would mix a change of speed with a change of
    distance; dividing a constrained run by an unconstrained one would mix in a
    different path. A case with no exact counterpart gets no ratio at all.
    """
    ref = baselines.get(config_key(case))
    if ref is None or ref[1] == case[3]:
        return None
    return ref


def config_key(case):
    """Everything about a case except its speed."""
    _group, _name, target_key, _speed, locks, approach, _expect = case
    return (target_key, tuple(locks), approach)


def what_to_watch(case, ref):
    """One line telling the operator what they should SEE, before it happens."""
    _group, _name, target_key, speed, locks, approach, expect = case
    leg = leg_shape(target_key)
    if expect == "refuse":
        held = "the other five pose components" if approach else ", ".join(locks)
        return f"nothing should move — holding {held} is impossible from START"
    if ref:
        return f"{leg}, at {ref[1] / speed:.0f}x the wall time of the same leg at {ref[1]}"
    if locks:
        # A position lock and an orientation lock look completely different on
        # the arm; telling someone to "watch it not rotate" for a held x is how
        # you get a false confirmation.
        if set(locks) <= _ROTATIONAL:
            cue = "watch the tool keep its orientation"
        elif set(locks).isdisjoint(_ROTATIONAL):
            cue = f"watch the tool stay in the same {'/'.join(sorted(locks))} plane"
        else:
            cue = "watch both the orientation and the held position axis"
        return f"{leg}, with {'+'.join(locks)} held — {cue}"
    if approach:
        return f"{leg}, easing into the goal over the last {round((1 - approach[2]) * 100)}%"
    return f"{leg}, unconstrained at full speed — this is the reference time"


def print_case_banner(i, total, case, baselines):
    _group, name, target_key, speed, locks, approach, expect = case
    tgt = _TARGETS[target_key]
    ref = baseline_for(case, baselines)
    print("\n" + "  " + "─" * 68)
    print(f"  CASE {i}/{total}   {name}")
    print(f"    leg        {target_key:<8} {START['pos']} -> {tgt['pos']}")
    print(f"    speed      {speed}")
    print(f"    locks      {'+'.join(locks) if locks else '(none)'}")
    print(
        f"    approach   {f'{approach[0]}m along {approach[1]} at {approach[2]}' if approach else '(none)'}"
    )
    print(f"    expect     {expect.upper()}")
    print(f"    watch      {what_to_watch(case, ref)}")
    if ref:
        print(
            f"    predict    ~{ref[0] * ref[1] / speed:.1f}s "
            f"(same leg at {ref[1]} took {ref[0]:.2f}s)"
        )


def report_case(case, code, wall, baselines):
    """Print the comparison for ONE case, right after it runs."""
    _group, _name, _target, speed, _locks, approach, expect = case
    ref = baseline_for(case, baselines)
    got = _CODES.get(code, str(code))
    moved = code == 0
    agreed = moved == (expect == "move")
    verdict = "AS EXPECTED" if agreed else "*** DISAGREES WITH EXPECTATION ***"
    print(f"\n    result     {got} in {wall:.2f}s  —  {verdict}")
    if moved and ref:
        ratio = wall / ref[0]
        ideal = ref[1] / speed
        print(
            f"    compare    {wall:.2f}s / {ref[0]:.2f}s (same leg at {ref[1]}) "
            f"= {ratio:.2f}x, ideal {ideal:.2f}x"
        )
        print(
            "               below ideal is expected: planning and the action "
            "round-trip are fixed cost and do not scale."
        )
    elif not agreed and expect == "refuse":
        print(
            "               it MOVED when the constraint should have been "
            "unsatisfiable from START — the expectation may be wrong, not the driver."
        )
    return agreed


def print_results(rows):
    """rows: (name, expect, code, wall, status) where status is ran|inconclusive."""
    print("\n  ==== results ====\n")
    head = f"  {'case':<24} {'expect':<7} {'got':<18} {'wall':>7}  {'ok':<12}"
    print(head)
    print("  " + "-" * (len(head) - 2))
    bad = 0
    incon = 0
    for name, expect, code, wall, status in rows:
        if status == "inconclusive":
            # NOT a pass and NOT a failure. The setup broke, so this case was
            # never tested -- scoring it against the expectation would have let
            # a broken re-home masquerade as a passing refusal.
            incon += 1
            print(
                f"  {name:<24} {expect:<7} {'(re-home failed)':<18} {'--':>7}  "
                f"{'INCONCLUSIVE':<12}"
            )
            continue
        got = _CODES.get(code, str(code))
        agreed = (code == 0) == (expect == "move")
        if not agreed:
            bad += 1
        print(
            f"  {name:<24} {expect:<7} {got:<18} {wall:>6.2f}s  "
            f"{'yes' if agreed else 'NO':<12}"
        )
    print()
    if incon:
        print(
            f"  {incon} case(s) INCONCLUSIVE — the re-home failed, so they never ran.\n"
            "  Fix the re-home before reading anything into the rest.\n"
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
    ap.add_argument(
        "--no-pause",
        action="store_true",
        help="do not wait for Enter between cases (unattended runs)",
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

    # Full-speed reference per CONFIGURATION: (leg, locks, approach) ->
    # (wall_seconds, speed). A case is only ever divided by a run that differs
    # from it in speed alone; anything else gets no ratio rather than a
    # misleading one.
    baselines = {}

    rows = []
    for i, case in enumerate(cases, 1):
        _group, name, target_key, speed, locks, approach, expect = case
        print_case_banner(i, len(cases), case, baselines)

        if not args.no_pause:
            try:
                ans = input("\n    [Enter] run   [s] skip   [q] quit: ").strip().lower()
            except EOFError:      # piped stdin: fall through and just run
                ans = ""
            if ans == "q":
                print("\n  stopped at the operator's request.\n")
                break
            if ans == "s":
                rows.append((name, expect, None, float("nan"), "inconclusive"))
                print("    skipped.")
                continue

        # Re-home unconstrained and at full speed so the timed leg always starts
        # from the same place. A failure here means the case never ran -- it is
        # INCONCLUSIVE, never a pass, however its code happens to compare.
        print("    re-homing...", end=" ", flush=True)
        home_code, home_wall = _send(
            node, client, _build_goal(START, args.sender_id, frame=args.frame), quiet=True
        )
        if home_code != 0:
            print(f"FAILED (code={home_code}) — case NOT run, recorded INCONCLUSIVE")
            rows.append((name, expect, home_code, float("nan"), "inconclusive"))
            continue
        print(f"ok ({home_wall:.2f}s)")

        print("    running...", end=" ", flush=True)
        goal = _build_goal(
            _TARGETS[target_key], args.sender_id, speed, locks, approach, args.frame
        )
        code, wall = _send(node, client, goal, quiet=True)
        print("done")
        report_case(case, code, wall, baselines)
        if code == 0 and speed == 1.0:
            baselines.setdefault(config_key(case), (wall, speed))
        rows.append((name, expect, code, wall, "ran"))

    rclpy.shutdown()
    bad, incon = print_results(rows)
    return 0 if (bad == 0 and incon == 0) else 3


if __name__ == "__main__":
    raise SystemExit(main())
