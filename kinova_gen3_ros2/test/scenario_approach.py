#!/usr/bin/env python3
"""One scenario, run twice: travel A -> B direct, then with an approach_offset.

The companion to scenario_carry_level.py, for the other constraint.

    run 1   direct     A -> B
    run 2   approach   A -> B, easing in along base Z from `offset` metres back

WHY A MULTI-AXIS MOVE, AFTER GETTING THIS WRONG ONCE. A lock and a via do NOT
share a rule, and conflating them made the first version of this scenario
incapable of showing anything:

    axis lock   hold_vec_weight, active the WHOLE trajectory
                => the start must already match the goal on every held axis
    via         grasp_approach_metric with tstep_fraction, active only over the
                FINAL fraction => the start does not have to match anything

Measured on the arm, same start and goal: a lock on a travelled axis is refused
(INVALID_PARTIAL_POSE_COST_METRIC in 0.02 s); an identical goal carrying a via
plans normally. So the earlier claim here -- that an approach implies holding the
other five and therefore needs a single-axis move -- was wrong, inherited from
the lock's semantics and never tested.

It also destroyed the demonstration. Forced onto a pure vertical descent, the via
had nothing to do: the direct path was ALREADY a straight line down the approach
axis, and the two runs came out 2.05 s vs 2.06 s with 19 mm vs 20 mm of drift.
A and B now differ in x, y AND z, which is the only geometry where a via has a
bend available to add.

WHAT TO WATCH. The lateral number is now the primary signal, not a control. The
direct run should track close to the straight A->B line; the approach run should
leave it, coming into B along base Z over the final stretch. If both stay equally
straight, the via is reaching cuRobo and changing nothing -- which would be the
real finding, and is exactly what this cannot currently distinguish on a
single-axis move.

Recorded per run, from /ee_state:
  * total wall time
  * the fraction of that time spent covering the final `offset` metres
  * the maximum lateral deviation from the straight line between the run's
    own measured endpoints

SAFETY: DRY RUN by default. --go moves the arm: 4 motions (to A, run 1, back to
A, run 2), stepping with Enter between each. Attended, e-stop in hand.

Examples:
    python3 scenario_approach.py                     # dry run
    python3 scenario_approach.py --go                # run it
    python3 scenario_approach.py --go --offset 0.15 --at-fraction 0.6
"""

import argparse
import math
import time

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rammp_arm_interfaces.action import GoToEEPose
    from rammp_arm_interfaces.msg import ApproachOffset, EeState, ToolAxisLock

    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False

# Gripper level: approach axis (tool Z) horizontal and facing forward, tool Y
# straight UP -- and that last part is the cup's axis. READ OFF THE ARM at a
# pose set by hand, not derived on paper. An earlier attempt computed a
# quaternion that also put tool Z forward but rolled the gripper 90 deg about
# it. That is the same "facing forward" in words and a wrist configuration
# cuRobo could not solve: 3 of 25 probed poses feasible, against 16 of 20 here.
GRIPPER_LEVEL = [0.5, 0.5, 0.5, 0.5]

# Sized for a DRAMATIC difference, which constrains the geometry more than it
# first appears. The via point is NOT free: cuRobo places it at
# B - offset * toolZ, so it always shares B's y and z and differs only along the
# tool axis (base +X at this orientation). The only way to make the detour large
# is to put A far off that line and make the offset long.
#
# A high and to one side, B far forward, low and to the other:
#   direct   A -> B        ~0.50 m
#   dog-leg  A -> via -> B ~0.70 m
# a 40% longer path, which is the thing that should be visible from across the
# room. Earlier attempts kept A, B and the via nearly collinear, so the "detour"
# was a few centimetres and no amount of staring at it would have helped.
POSE_A = {"name": "A", "pos": [0.50, -0.20, 0.55], "quat": list(GRIPPER_LEVEL)}
POSE_B = {"name": "B", "pos": [0.65, 0.20, 0.30], "quat": list(GRIPPER_LEVEL)}

# How far back along the tool axis the approach starts. The via POSITION is
# derived from this below rather than written down, because a hand-written via
# can disagree with B and there is no way for cuRobo to honour it if it does.
APPROACH_OFFSET_M = 0.22

_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}


def tool_axis(q_xyzw, axis=2):
    """The tool's local axis `axis` (0=x,1=y,2=z) in the base frame."""
    x, y, z, w = q_xyzw
    cols = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)),
        (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)),
        (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)),
    )
    return cols[axis]


def via_position(goal_pos, quat, offset):
    """Where cuRobo's pregrasp actually sits: back along the TOOL's z axis.

    Derived, never written down. The via is not a free waypoint -- it is fixed
    by the goal, the offset and the goal's own orientation -- so a hand-written
    coordinate can only ever agree with it by luck.
    """
    ax = tool_axis(quat, 2)
    return [goal_pos[i] - offset * ax[i] for i in range(3)]


# ---------------------------------------------------------------- measurement
# Module-level and ROS-free on purpose, so they can be checked without an arm.


def lateral_deviation(p, a, b):
    """Perpendicular distance from point p to the infinite line through a, b."""
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    ab_len = math.sqrt(sum(c * c for c in ab))
    if ab_len < 1e-9:
        return 0.0
    cross = (
        ap[1] * ab[2] - ap[2] * ab[1],
        ap[2] * ab[0] - ap[0] * ab[2],
        ap[0] * ab[1] - ap[1] * ab[0],
    )
    return math.sqrt(sum(c * c for c in cross)) / ab_len


def final_stretch_fraction(samples, goal, offset):
    """Fraction of the run's elapsed time spent within `offset` metres of goal.

    samples: [(t_seconds, (x, y, z)), ...] in order. Returns None if the run
    never got that close, which would mean it did not actually arrive.

    Discretization biases this LOW by up to one sample interval, since it can
    only report the first sample already inside the radius. At /ee_state's rate
    over a motion of a second or more that is well under a percentage point --
    which is why the comparison below refuses to call a difference under five.
    """
    if len(samples) < 2:
        return None
    t_start, t_end = samples[0][0], samples[-1][0]
    span = t_end - t_start
    if span <= 0:
        return None
    for t, p in samples:
        if math.dist(p, goal) <= offset:
            return (t_end - t) / span
    return None


def summarize(samples, offset):
    """(wall_span, final_fraction, max_lateral) from a run's own samples.

    Endpoints come from the SAMPLES, not from the commanded coordinates. That is
    deliberate. /ee_state reports `gen3_end_effector_link` out of our urdf, while
    the goal we send is cuRobo's `tool_frame` out of NVIDIA's bundled model --
    different frames from different models, about 12 cm apart along the tool
    axis. Measuring the reported position against a commanded coordinate mixes
    the two and produces a confident, meaningless number; the first version of
    this script reported 120 mm of "lateral drift" on a run where the arm never
    moved, which is exactly that error. Measuring a run against its own first and
    last sample is frame-agnostic and needs no transform.
    """
    if len(samples) < 2:
        return None, None, None
    span = samples[-1][0] - samples[0][0]
    a, b = samples[0][1], samples[-1][1]
    worst = max(lateral_deviation(p, a, b) for _t, p in samples)
    return span, final_stretch_fraction(samples, b, offset), worst


if _HAVE_ROS:

    class Scenario(Node):
        def __init__(self, sender_id):
            super().__init__("scenario_approach")
            self.sender_id = sender_id
            self.client = ActionClient(self, GoToEEPose, "go_to_ee_pose")
            self.samples = []
            self.sampling = False
            self.create_subscription(
                EeState, "/ee_state", self._on_ee, qos_profile_sensor_data
            )

        def _on_ee(self, msg):
            if not self.sampling:
                return
            p = msg.pose.position
            self.samples.append((time.monotonic(), (p.x, p.y, p.z)))

        def goal(self, pose, speed=1.0, approach=None, at_fraction=0.8):
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
            ) = pose["quat"]
            g.sender_id = self.sender_id
            g.speed_scale = speed
            g.axis_lock = ToolAxisLock()
            # Base frame: AXIS_Z is then the world vertical, i.e. from above.
            g.axis_lock.reference_frame = ToolAxisLock.FRAME_BASE
            g.approach_offset = ApproachOffset()
            if approach:
                g.approach_offset.distance = approach
                g.approach_offset.axis = ApproachOffset.AXIS_Z
                g.approach_offset.at_fraction = at_fraction
            return g

        def send(self, goal, measure=False):
            """Returns (code, wall_seconds, samples)."""
            self.samples = []
            self.sampling = measure
            t0 = time.monotonic()
            fut = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.sampling = False
                return -1, time.monotonic() - t0, []
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf)
            self.sampling = False
            return (
                rf.result().result.error_code,
                time.monotonic() - t0,
                list(self.samples),
            )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--speed", type=float, default=0.5, help="speed_scale for BOTH runs")
    ap.add_argument(
        "--offset",
        type=float,
        default=None,
        help="metres back along the tool Z axis; defaults to the distance from B "
        "to APPROACH_VIA, so the pregrasp lands on that known-feasible pose",
    )
    ap.add_argument(
        "--at-fraction",
        type=float,
        default=None,
        help="when the hold engages; defaults to where APPROACH_VIA actually "
        "falls along A -> via -> B, because a fraction inconsistent with the "
        "geometry asks the arm to idle and then rush",
    )
    ap.add_argument("--sender-id", default="scenario_approach")
    ap.add_argument("--no-pause", action="store_true")
    ap.add_argument("--go", action="store_true", help="ACTUALLY MOVE THE ARM")
    args = ap.parse_args()

    if not 0.01 <= args.speed <= 1.0:
        ap.error(f"--speed must be in [0.01, 1.0]; got {args.speed}")
    if args.offset is None:
        args.offset = APPROACH_OFFSET_M
    if args.offset <= 0.0:
        ap.error("--offset must be > 0")
    via = via_position(POSE_B["pos"], POSE_B["quat"], args.offset)
    if args.at_fraction is None:
        # Where the via really sits along A -> via -> B. Pinning the hold at 0.8
        # while the via sits at half the path told the arm to cover 20% of the
        # distance in 80% of the time and then sprint -- geometry and schedule
        # have to agree or the request is self-contradictory.
        legs = math.dist(POSE_A["pos"], via)
        args.at_fraction = round(legs / (legs + args.offset), 3)
    if not 0.0 < args.at_fraction < 1.0:
        ap.error("--at-fraction must be strictly inside (0, 1)")
    span = math.dist(POSE_A["pos"], POSE_B["pos"])
    # Compared against the DOG-LEG length, not the direct span: the whole point
    # of a forward goal is that the offset may exceed the straight-line distance.
    dogleg = math.dist(POSE_A["pos"], via) + args.offset
    if args.offset >= dogleg:
        ap.error(f"--offset {args.offset} must be less than the {dogleg:.2f} m path")

    print(f"\n  A  {POSE_A['pos']}   gripper level, facing forward")
    print(f"  B  {POSE_B['pos']}   gripper level, facing forward")
    print(f"  a {span:.2f} m move with x, y and z all differing — the only geometry")
    print("  where a via has a bend available to add")
    print(f"  via point  [{via[0]:.3f}, {via[1]:.3f}, {via[2]:.3f}]"
          f"   (B minus {args.offset} m along the tool Z axis — derived, not chosen)")
    print(f"  direct A -> B      {span:.2f} m")
    print(f"  dog-leg A -> via -> B  {dogleg:.2f} m"
          f"   ({dogleg / span - 1.0:+.0%} longer — this is what should be visible)\n")
    print(f"  run 1   direct     A -> B at speed_scale {args.speed}")
    print(
        f"  run 2   approach   A -> B at speed_scale {args.speed}, "
        f"{args.offset} m back along Z, engaging at {args.at_fraction}\n"
    )
    print("  measured per run: wall time, the share of it spent inside the final")
    print(f"  {args.offset} m, and the worst lateral drift off the straight line")
    print("  between the run's own measured endpoints. NOTE /ee_state reports a")
    print("  frame ~12 cm behind the one we command (see summarize()), so every")
    print("  number below is relative to the run itself, never to A and B.\n")

    if not args.go:
        print("  DRY RUN — nothing was sent. Pass --go to move the arm.\n")
        return 0
    if not _HAVE_ROS:
        print("  --go needs rclpy + rammp_arm_interfaces on the path.\n")
        return 2

    def pause(what):
        if args.no_pause:
            return True
        try:
            return input(f"    [Enter] {what}   [q] quit: ").strip().lower() != "q"
        except EOFError:
            return True

    rclpy.init()
    n = Scenario(args.sender_id)
    if not n.client.wait_for_server(timeout_sec=5.0):
        n.get_logger().error("go_to_ee_pose action server not available")
        rclpy.shutdown()
        return 1

    results = {}
    try:
        for label, approach in (("direct", None), ("approach", args.offset)):
            print("\n  ── setting up: moving to A")
            if not pause("move to A"):
                break
            code, wall, _ = n.send(n.goal(POSE_A))
            if code != 0:
                print(f"    could not reach A ({_CODES.get(code, code)}) — stopping.")
                print("    if this is unreachable at the level gripper pose, pull")
                print("    POSE_A/POSE_B's x in toward the base and retry.")
                return 3
            print(f"    at A ({wall:.2f}s)")

            print(f"\n  ── run: {label}   travel A -> B at {args.speed}")
            if not pause(f"run {label}"):
                break
            code, wall, samples = n.send(
                n.goal(POSE_B, args.speed, approach, args.at_fraction), measure=True
            )
            span, frac, lat = summarize(samples, args.offset)
            print(f"    {_CODES.get(code, code)} in {wall:.2f}s, {len(samples)} samples")
            if frac is not None:
                print(
                    f"    final {args.offset} m took {frac * 100:.0f}% of the motion; "
                    f"worst lateral drift {lat * 1000:.0f} mm"
                )
            results[label] = (code, wall, span, frac, lat, len(samples))
    finally:
        rclpy.shutdown()

    print("\n  ==== comparison ====\n")
    print(f"  {'run':<10} {'result':<14} {'wall':>7} {'final ' + str(args.offset) + 'm':>12} {'lateral':>9}")
    print("  " + "-" * 58)
    for label in ("direct", "approach"):
        if label not in results:
            print(f"  {label:<10} {'(not run)':<14} {'--':>7} {'--':>12} {'--':>9}")
            continue
        code, wall, _span, frac, lat, _n = results[label]
        f_s = f"{frac * 100:.0f}%" if frac is not None else "n/a"
        l_s = f"{lat * 1000:.0f} mm" if lat is not None else "n/a"
        print(f"  {label:<10} {_CODES.get(code, code):<14} {wall:>6.2f}s {f_s:>12} {l_s:>9}")
    print()

    if len(results) == 2 and all(r[0] == 0 for r in results.values()):
        fd, fa = results["direct"][3], results["approach"][3]
        if fd is None or fa is None:
            print("  not enough /ee_state samples inside the final stretch to compare.\n")
        elif abs(fa - fd) < 0.05:
            print(
                "  the two schedules are within 5 percentage points — on this pose\n"
                "  pair the approach changed the timing little or not at all. Try a\n"
                "  larger --offset or a lower --at-fraction before concluding anything.\n"
            )
        else:
            slower = "more" if fa > fd else "less"
            print(
                f"  the approach spent {slower} of the motion in the final "
                f"{args.offset} m ({fa * 100:.0f}% vs {fd * 100:.0f}%).\n"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
