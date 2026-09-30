#!/usr/bin/env python3
"""One scenario, run twice: carry the tool A -> B, free, then held level.

This is the "don't spill the cup" demo. The SAME motion at the SAME speed runs
twice, and the only difference is whether roll and pitch are held:

    run 1   free     A -> B at speed_scale 0.5
    run 2   level    A -> B at speed_scale 0.5, roll+pitch held in the BASE frame

A and B are at the SAME tool orientation -- the gripper LEVEL, approach axis
horizontal and facing forward, gripping a cup from the side with the cup's axis
vertical -- and differ only in position. That matters: a lock holds a component
AT THE GOAL'S VALUE, so roll/pitch can only be held if the start already matches
the goal on them. It also means the difference to watch is in the MIDDLE of the
motion -- unconstrained, cuRobo is free to tilt the tool on its way between two
level poses; constrained, it is not.

"Level" is gravity-relative, so the lock is applied in FRAME_BASE. In FRAME_GOAL
it would be held relative to the goal's own frame, which is a different question.

MEASURED, not eyeballed: /ee_state is sampled throughout each run and the script
reports the worst tilt of the CUP's axis away from vertical. Which tool axis that
is depends on the grasp pose (it is the tool's local X here, not its Z), so it is
derived from the reference orientation rather than assumed. Expect a
few degrees or more on the free run and ~0 on the held one. If both come back
near zero the planner simply chose a level path anyway -- that is not a
demonstration of anything, so push A and B further apart and rerun.

SAFETY: DRY RUN by default. --go moves the arm: 4 motions in total (a move to A,
run 1, back to A, run 2). Attended, e-stop in hand. A and B default to the pose
pair the constraint sweep already exercises on this cell.

Examples:
    python3 scenario_carry_level.py                  # dry run
    python3 scenario_carry_level.py --go             # run it
    python3 scenario_carry_level.py --go --speed 0.25
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

# Gripper LEVEL, approach axis horizontal and facing forward (+X): the cup is
# gripped from the side with its axis vertical. xyzw; a +90 deg rotation about Y.
#
# In this pose the tool's local axes land as: Z -> +X (forward), Y -> +Y, and
# X -> straight DOWN. So the axis that must stay vertical -- the cup's axis, the
# one that decides whether it spills -- is the tool's local X, not its Z. That is
# derived below rather than hardcoded, so changing this constant keeps the
# measurement honest.
GRIPPER_LEVEL = [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]
# Same orientation at both ends -- see the module docstring for why that is the
# whole point. Reuses the sweep's known-good pair for this cell.
POSE_A = {"name": "A", "pos": [0.45, -0.25, 0.25], "quat": list(GRIPPER_LEVEL)}
POSE_B = {"name": "B", "pos": [0.45, 0.25, 0.50], "quat": list(GRIPPER_LEVEL)}

_CODES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -4: "PATH_TOLERANCE_VIOLATED",
    -6: "PREEMPTED",
    -7: "PLANNING_FAILED",
    -8: "NOT_AUTHORIZED",
    -9: "HALTED",
}


def rotate_axis(q_xyzw, axis):
    """A local unit axis ('x'|'y'|'z') expressed in the base frame, for q."""
    x, y, z, w = q_xyzw
    if axis == "x":
        return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y))
    if axis == "y":
        return (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x))
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))


def spill_axis(reference_q):
    """Which LOCAL tool axis is vertical in the reference pose.

    That axis is the cup's axis, and its deviation from vertical is what decides
    whether the cup spills. Derived from the reference orientation instead of
    assumed, so a different grasp pose does not silently leave the measurement
    tracking the wrong axis -- which is exactly the mistake this replaces.
    """
    return max("xyz", key=lambda a: abs(rotate_axis(reference_q, a)[2]))


# The reference orientation is CALIBRATED from the arm at pose A, not taken from
# GRIPPER_LEVEL. /ee_state reports `gen3_end_effector_link` out of our urdf while
# the goal we send is cuRobo's `tool_frame` out of NVIDIA's bundled model --
# different frames from different models, and the urdf chain to our flange
# carries a 180 deg roll. Comparing a measured orientation against a commanded
# one therefore mixes conventions and reports a tilt that is wrong by a fixed
# rotation. Observing the arm at a known-level pose sidesteps every bit of that.


def tilt_from_level_deg(q_xyzw, reference_q=GRIPPER_LEVEL, axis=None):
    """Degrees the cup's axis has tipped away from where the reference puts it.

    One number rather than separate roll and pitch, and rotation ABOUT the cup's
    own axis contributes nothing -- correct, because spinning a cup on its axis
    does not spill it and that rotation is not locked either.
    """
    axis = axis or spill_axis(reference_q)
    a = rotate_axis(q_xyzw, axis)
    b = rotate_axis(reference_q, axis)
    dot = sum(i * j for i, j in zip(a, b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _normalize(quat):
    n = math.sqrt(sum(c * c for c in quat))
    if n < 1e-9:
        raise ValueError(f"degenerate quaternion {quat}")
    return [c / n for c in quat]


# Defined only when ROS is importable: the class body evaluates at import time
# and would crash a dry run on a machine with no ROS, which is exactly where the
# dry run is meant to be useful.
if _HAVE_ROS:

  class Scenario(Node):
      def __init__(self, sender_id):
          super().__init__("scenario_carry_level")
          self.sender_id = sender_id
          self.client = ActionClient(self, GoToEEPose, "go_to_ee_pose")
          self.tilts = []
          self.sampling = False
          self.ref_q = None       # calibrated at A; see the note by GRIPPER_LEVEL
          self.ref_axis = None
          self.latest_q = None
          self.create_subscription(
              EeState, "/ee_state", self._on_ee, qos_profile_sensor_data
          )

      def _on_ee(self, msg):
          o = msg.pose.orientation
          self.latest_q = [o.x, o.y, o.z, o.w]
          if not self.sampling or self.ref_q is None:
              return
          self.tilts.append(
              tilt_from_level_deg(self.latest_q, self.ref_q, self.ref_axis)
          )

      def calibrate(self, settle=1.0):
          """Adopt the arm's CURRENT orientation as level. Called at pose A.

          This is what makes the measurement frame-agnostic: the reference comes
          from the same topic, in the same convention, as everything compared
          against it.
          """
          end = time.monotonic() + settle
          while time.monotonic() < end and self.latest_q is None:
              rclpy.spin_once(self, timeout_sec=0.05)
          if self.latest_q is None:
              return False
          self.ref_q = list(self.latest_q)
          self.ref_axis = spill_axis(self.ref_q)
          return True

      def goal(self, pose, speed=1.0, level=False):
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
          g.sender_id = self.sender_id
          g.speed_scale = speed
          g.axis_lock = ToolAxisLock()
          # Gravity-relative: "level" only means anything in the base frame.
          g.axis_lock.reference_frame = ToolAxisLock.FRAME_BASE
          if level:
              g.axis_lock.lock_roll = True
              g.axis_lock.lock_pitch = True
          g.approach_offset = ApproachOffset()
          return g

      def send(self, goal, measure=False):
          """Returns (code, wall_seconds, max_tilt_deg or None)."""
          self.tilts = []
          self.sampling = measure
          t0 = time.monotonic()
          fut = self.client.send_goal_async(goal)
          rclpy.spin_until_future_complete(self, fut)
          gh = fut.result()
          if gh is None or not gh.accepted:
              self.sampling = False
              return -1, time.monotonic() - t0, None
          rf = gh.get_result_async()
          rclpy.spin_until_future_complete(self, rf)
          wall = time.monotonic() - t0
          self.sampling = False
          code = rf.result().result.error_code
          worst = max(self.tilts) if self.tilts else None
          return code, wall, worst


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--speed", type=float, default=0.5, help="speed_scale for BOTH runs")
    ap.add_argument("--sender-id", default="scenario_carry_level")
    ap.add_argument("--no-pause", action="store_true", help="do not wait for Enter")
    ap.add_argument("--go", action="store_true", help="ACTUALLY MOVE THE ARM")
    args = ap.parse_args()

    if not 0.01 <= args.speed <= 1.0:
        ap.error(f"--speed must be in [0.01, 1.0]; got {args.speed}")

    dist = math.dist(POSE_A["pos"], POSE_B["pos"])
    print(f"\n  A  {POSE_A['pos']}   gripper level, facing forward")
    print(f"  B  {POSE_B['pos']}   gripper level, facing forward")
    print(f"  {dist:.2f} m apart, same orientation at both ends\n")
    print(f"  run 1   free    A -> B at speed_scale {args.speed}")
    print(f"  run 2   level   A -> B at speed_scale {args.speed}, roll+pitch held (base frame)\n")
    print("  measured: worst tilt of the cup's axis away from the reference the")
    print("  arm itself reports at A, sampled from /ee_state throughout each run.")
    print("  Calibrated, not assumed: the measured frame and the commanded frame")
    print("  come from different robot models (see the note by GRIPPER_LEVEL).\n")

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
        for label, level in (("free", False), ("level", True)):
            print(f"\n  ── setting up: moving to A")
            if not pause("move to A"):
                break
            code, wall, _ = n.send(n.goal(POSE_A))
            if code != 0:
                print(f"    could not reach A ({_CODES.get(code, code)}) — stopping.")
                return 3
            if not n.calibrate():
                print("    no /ee_state — cannot calibrate the level reference.")
                return 3
            print(f"    at A ({wall:.2f}s); level reference calibrated from the arm, "
                  f"cup axis = tool {n.ref_axis.upper()}")

            print(f"\n  ── run: {label}   A -> B at {args.speed}"
                  + ("   roll+pitch HELD" if level else "   unconstrained"))
            if not pause(f"run {label}"):
                break
            code, wall, worst = n.send(n.goal(POSE_B, args.speed, level), measure=True)
            label_code = _CODES.get(code, str(code))
            tilt_s = f"{worst:.1f}°" if worst is not None else "no /ee_state samples"
            print(f"    {label_code} in {wall:.2f}s   worst tilt {tilt_s}")
            results[label] = (code, wall, worst)
    finally:
        rclpy.shutdown()

    print("\n  ==== comparison ====\n")
    print(f"  {'run':<8} {'result':<16} {'wall':>7}  {'worst tilt':>11}")
    print("  " + "-" * 48)
    for label in ("free", "level"):
        if label not in results:
            print(f"  {label:<8} {'(not run)':<16} {'--':>7}  {'--':>11}")
            continue
        code, wall, worst = results[label]
        t = f"{worst:.1f}°" if worst is not None else "n/a"
        print(f"  {label:<8} {_CODES.get(code, code):<16} {wall:>6.2f}s  {t:>11}")
    print()

    if len(results) == 2 and all(r[0] == 0 for r in results.values()):
        f, lv = results["free"][2], results["level"][2]
        if f is None or lv is None:
            print("  no /ee_state samples — cannot compare tilt.\n")
        elif f - lv > 2.0:
            print(f"  the lock held the tool {f - lv:.1f}° flatter through the carry.\n")
        else:
            print(
                f"  free {f:.1f}° vs level {lv:.1f}° — too close to call. The planner\n"
                "  likely chose a level path anyway, so this run demonstrates nothing;\n"
                "  move A and B further apart and try again.\n"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
