#!/usr/bin/env python3
"""Render the arm's model from a mount file: xacro, then strip <ros2_control>.

The ONE place the model is rendered. description.launch.py calls render() at launch
and publishes the result on /arm/robot_description; the driver and cuRobo read that
topic, so nothing else renders, and there is no flat model on disk to pick up by
mistake.

Why <ros2_control> is stripped: kortex_description's load_arm emits one
unconditionally, declaring kortex_driver's hardware interface. That is not merely
untrue for us, it is a hazard: a controller_manager pointed at our description would
load that plugin, open a SECOND direct KORTEX connection, and command the arm alongside
our driver -- bypassing arbitration entirely, since arbitration gates the one process
it lives in. robot_state_publisher and Pinocchio both ignore the block, so removing it
costs nothing.

Bench use (the driver's standalone apps take --urdf):
    ros2 run kinova_gen3_description kinova_gen3_render.py \
        --mount /config/robot/mount.yaml --out /tmp/arm.urdf
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

MOUNT_KEYS = {"robot", "rpy_deg"}


def load_mount(path):
    """Read and check a mount.yaml. Raises ValueError naming the file and the problem."""
    with open(path) as f:
        mount = yaml.safe_load(f)
    if not isinstance(mount, dict):
        raise ValueError(f"{path}: expected a mapping with keys {sorted(MOUNT_KEYS)}")
    unknown = set(mount) - MOUNT_KEYS
    missing = MOUNT_KEYS - set(mount)
    if unknown or missing:
        raise ValueError(
            f"{path}: unknown keys {sorted(unknown)}, missing keys {sorted(missing)}"
        )
    rpy = mount["rpy_deg"]
    if (
        not isinstance(rpy, list)
        or len(rpy) != 3
        or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in rpy
        )
    ):
        raise ValueError(f"{path}: rpy_deg must be three finite numbers, got {rpy!r}")
    robot = mount["robot"]
    if not isinstance(robot, str) or not robot.strip():
        raise ValueError(f"{path}: robot must be a non-empty name, got {robot!r}")
    # rpy_deg is DEGREES. Radians typed in by mistake (0.5236 for 30 deg) render a
    # plausible-looking, nearly level mount -- and the wrong gravity -- so say so.
    if any(rpy) and all(abs(v) <= 2 * math.pi for v in rpy):
        print(
            f"WARNING: {path}: rpy_deg {rpy} is read as DEGREES, but every value is "
            "within 2*pi -- radians? Rendering it as degrees.",
            file=sys.stderr,
        )
    return mount


def embodiment_line(mount):
    """The first thing the description logs, so the active robot is never a guess."""
    r, p, y = (float(v) for v in mount["rpy_deg"])
    return f"embodiment: {mount['robot']} (mount rpy_deg [{r}, {p}, {y}])"


def render(mount_file):
    """Return (urdf_xml, mount). Raises on a bad mount file or a xacro failure."""
    import xacro  # deferred: importing it is slow and --help should not pay for it
    from ament_index_python.packages import get_package_share_directory

    mount = load_mount(mount_file)
    src = os.path.join(
        get_package_share_directory("kinova_gen3_description"),
        "urdf",
        "kinova_gen3.urdf.xacro",
    )
    doc = xacro.process_file(src, mappings={"mount_file": os.path.abspath(mount_file)})
    root = ET.fromstring(doc.toxml())
    for block in root.findall("ros2_control"):
        root.remove(block)
    return ET.tostring(root, encoding="unicode"), mount


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mount", required=True, help="path to the robot's mount.yaml")
    ap.add_argument("--out", required=True, help="where to write the URDF")
    args = ap.parse_args(argv)
    xml, mount = render(args.mount)
    with open(args.out, "w") as f:
        f.write(xml)
    print(f"{args.out}: {embodiment_line(mount)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
