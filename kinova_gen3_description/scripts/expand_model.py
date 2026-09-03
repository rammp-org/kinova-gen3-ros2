#!/usr/bin/env python3
"""Expand the model xacro into a plain URDF, applying two transforms xacro cannot.

TWO transforms, for two different reasons:

1. Strip <ros2_control>. kortex_description's load_arm emits one unconditionally,
   declaring kortex_driver's hardware interface. That is not merely untrue for us, it is
   a hazard: a controller_manager pointed at our /robot_description would load that
   plugin, open a SECOND direct KORTEX connection, and command the arm alongside our
   driver -- bypassing arbitration entirely, since arbitration gates the one process it
   lives in. robot_state_publisher and Pinocchio both ignore the block, so removing it
   costs nothing.

2. --freeze-gripper converts the six Robotiq joints to type="fixed", producing a 7-DOF
   model. Core's Dynamics asserts nv == kNumJoints == 7 and refuses anything else:

       Dynamics: URDF nv=13 != kNumJoints=7 (wrong URDF for this build)

   JointVec is a fixed-size 7-vector, so a 13-DOF model has nowhere to go. Pinocchio
   lumps a fixed joint's body into its parent, so the frozen model keeps the gripper's
   MASS at the wrist -- which is what gravity compensation needs -- while dropping its
   degrees of freedom. This is almost certainly why the hand-edited model we are
   replacing had every Robotiq joint fixed: it was a workaround, not an oversight.

So we ship both: the articulated model for robot_state_publisher (RViz needs the fingers
to move) and the frozen one for the driver.
"""
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

# The one actuated joint plus its five mimics. Frozen together or not at all: freezing a
# subset would leave mimic joints referencing a fixed joint, which is malformed.
GRIPPER_JOINTS = (
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--freeze-gripper", action="store_true",
                    help="convert the Robotiq joints to fixed, giving a 7-DOF model")
    ap.add_argument("xacro_args", nargs="*")
    args = ap.parse_args()

    # Do NOT swallow stderr on failure: xacro's diagnostics are the only thing that says
    # WHICH macro rejected WHICH parameter, and a traceback without them is unactionable.
    proc = subprocess.run(["xacro", args.src, *args.xacro_args],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        return proc.returncode

    root = ET.fromstring(proc.stdout)

    stripped = 0
    for block in root.findall("ros2_control"):
        root.remove(block)
        stripped += 1

    frozen = 0
    if args.freeze_gripper:
        for joint in root.findall("joint"):
            if joint.get("name") in GRIPPER_JOINTS:
                joint.set("type", "fixed")
                # A fixed joint may carry neither <mimic>, <axis> nor <limit>; leaving
                # them makes the URDF malformed for strict parsers even though Pinocchio
                # tolerates it.
                for tag in ("mimic", "axis", "limit", "dynamics"):
                    for child in joint.findall(tag):
                        joint.remove(child)
                frozen += 1
        if frozen != len(GRIPPER_JOINTS):
            print(f"error: froze {frozen} gripper joints, expected {len(GRIPPER_JOINTS)}"
                  " -- upstream joint names changed?", file=sys.stderr)
            return 1

    ET.ElementTree(root).write(args.dst, encoding="utf-8", xml_declaration=True)
    print(f"{args.dst}: stripped {stripped} ros2_control block(s), "
          f"froze {frozen} gripper joint(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
