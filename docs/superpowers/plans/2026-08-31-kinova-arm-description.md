# `kinova_gen3_description` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A sister package holding a regenerable robot model, `/robot_description`, TF via
`robot_state_publisher`, and launch files — fixing the silent TF breakage caused by a
joint-name mismatch.

**Architecture:** A xacro composing `kortex_description`'s `load_arm` with
`robotiq_description`'s `robotiq_gripper` directly, bypassing kortex's `load_robot` shim
(broken against the installed robotiq). Expanded to plain URDF at build time for core's
Pinocchio, with the arm's `ros2_control` block stripped.

**Tech Stack:** ROS 2 Humble, `xacro`, `robot_state_publisher`, `ament_cmake`, Pinocchio
(for the parity check), Python 3.

**Spec:** `docs/superpowers/specs/2026-08-31-kinova-arm-description-design.md`

## Global Constraints

- **Joint names are unprefixed:** `joint_1 … joint_7`, matching what `Ros2Backend`
  already publishes. The `gen3_` prefix does not come back.
- **Do not use kortex's `load_robot` or `load_gripper`.** Installed `kortex_description`
  0.2.3 passes `isaac_joint_commands` / `isaac_joint_states` to
  `robotiq_description`'s `robotiq_gripper`, which accepts neither. Compose `load_arm`
  and `robotiq_gripper` directly.
- **The gripper's `ros2_control` is suppressed** with `include_ros2_control:=false`. The
  arm's has no opt-out and is stripped post-expansion.
- **No meshes are vendored.** The three upstream description packages are apt-installable
  and already present on abra.
- **A model difference is a finding, not a tolerance to widen.** The parity check compares
  the generated model against the current one at `1e-9`; a mismatch stops the task and
  gets reported with numbers.
- **Build/test:** `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2`, then
  `ssh abra "/tmp/rtest.sh"`. Use `--packages-up-to`, never `--packages-select`.

---

### Task 1: Package skeleton and the composed xacro

**Files:**
- Create: `kinova_gen3_description/package.xml`
- Create: `kinova_gen3_description/CMakeLists.txt`
- Create: `kinova_gen3_description/urdf/kinova_gen3.urdf.xacro`
- Create: `kinova_gen3_description/scripts/strip_ros2_control.py`

**Interfaces:**
- Consumes: nothing in this repo.
- Produces: package `kinova_gen3_description`, installing
  `share/kinova_gen3_description/urdf/kinova_gen3.urdf.xacro` and a build-time-expanded
  `share/kinova_gen3_description/urdf/kinova_gen3.urdf`. Later tasks reference both paths.

- [ ] **Step 1: Write `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>kinova_gen3_description</name>
  <version>0.1.0</version>
  <description>Kinova Gen3 7-DOF robot model, composed from upstream descriptions.</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <!-- xacro is needed at BUILD time: CMakeLists expands the model for core's Pinocchio,
       which cannot read xacro. -->
  <build_depend>xacro</build_depend>
  <build_depend>kortex_description</build_depend>
  <build_depend>robotiq_description</build_depend>
  <build_depend>realsense2_description</build_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>kortex_description</exec_depend>
  <exec_depend>robotiq_description</exec_depend>
  <exec_depend>realsense2_description</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: Write the composed xacro**

`kinova_gen3_description/urdf/kinova_gen3.urdf.xacro`:
```xml
<?xml version="1.0"?>
<!--
  The Gen3 7-DOF, composed from upstream descriptions rather than hand-edited.

  NOT built on kortex_description's load_robot/load_gripper: that shim passes
  isaac_joint_commands / isaac_joint_states down to robotiq_description's
  robotiq_gripper macro, which accepts neither (kortex_description 0.2.3 against the
  robotiq_description in Humble). Composing load_arm and robotiq_gripper directly
  sidesteps the version skew and costs nothing -- the shim adds no geometry.

  Joints come out UNPREFIXED: joint_1 .. joint_7, which is what Ros2Backend publishes
  on /joint_states and what every other Kinova stack uses. The gen3_ prefix in the old
  hand-edited model came from a Clearpath mobile-base workspace and meant nothing here.
-->
<robot name="kinova_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:arg name="gripper" default="robotiq_2f_85"/>   <!-- robotiq_2f_85 | none -->
  <xacro:arg name="camera"  default="true"/>            <!-- wrist vision module -->
  <xacro:arg name="prefix"  default=""/>                <!-- for a future mobile base -->

  <link name="world"/>

  <xacro:include filename="$(find kortex_description)/arms/gen3/7dof/urdf/gen3_macro.xacro"/>
  <xacro:include filename="$(find robotiq_description)/urdf/robotiq_2f_85_macro.urdf.xacro"/>

  <!-- load_arm REQUIRES connection parameters because it emits a ros2_control block we
       do not want and cannot opt out of. They are dummies: nothing here connects to an
       arm, and the block is stripped from the expanded URDF at build time. See
       scripts/strip_ros2_control.py for why that matters. -->
  <xacro:load_arm parent="world" dof="7" vision="$(arg camera)" prefix="$(arg prefix)"
      robot_ip="0.0.0.0" username="admin" password="admin"
      port="10000" port_realtime="10001"
      session_inactivity_timeout_ms="6000" connection_inactivity_timeout_ms="2000"
      gripper_joint_name="$(arg prefix)robotiq_85_left_knuckle_joint"
      use_internal_bus_gripper_comm="false">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:load_arm>

  <!-- include_ros2_control=false: the gripper macro DOES offer the opt-out the arm
       macro lacks, so this half emits geometry only. -->
  <xacro:if value="${'$(arg gripper)' == 'robotiq_2f_85'}">
    <xacro:robotiq_gripper name="RobotiqGripper" prefix="$(arg prefix)"
        parent="$(arg prefix)end_effector_link" include_ros2_control="false">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:robotiq_gripper>
  </xacro:if>

</robot>
```

- [ ] **Step 3: Write the `ros2_control` stripper**

`kinova_gen3_description/scripts/strip_ros2_control.py`:
```python
#!/usr/bin/env python3
"""Expand a xacro and remove <ros2_control> blocks from the result.

kortex_description's load_arm emits a <ros2_control> block unconditionally, declaring
kortex_driver's hardware interface. That is not merely untrue for us -- it is a hazard.
A controller_manager pointed at our /robot_description would load that plugin, open a
SECOND direct KORTEX connection, and command the arm alongside our driver. It would
bypass the arbitration tier completely, because arbitration gates the one process it
lives in and this would be another one talking straight to the hardware.

robot_state_publisher and Pinocchio both ignore the block, so stripping costs nothing
and removes the trap.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: strip_ros2_control.py <in.xacro> <out.urdf> [xacro args...]",
              file=sys.stderr)
        return 2
    src, dst, extra = sys.argv[1], sys.argv[2], sys.argv[3:]

    expanded = subprocess.run(["xacro", src, *extra], check=True,
                              capture_output=True, text=True).stdout
    root = ET.fromstring(expanded)
    removed = 0
    for block in root.findall("ros2_control"):
        root.remove(block)
        removed += 1
    ET.ElementTree(root).write(dst, encoding="utf-8", xml_declaration=True)
    print(f"{dst}: wrote model, stripped {removed} ros2_control block(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Write `CMakeLists.txt` that expands at build time**

```cmake
cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_description)
find_package(ament_cmake REQUIRED)

# Expand the xacro at BUILD time. Core's Pinocchio loads a URDF path and cannot read
# xacro, and expanding at build time (rather than checking the result in) keeps the
# upstream mesh paths -- which resolve to file:///opt/ros/... -- correct for whatever
# machine actually built it.
find_program(XACRO_EXE xacro REQUIRED)
set(MODEL_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/urdf/kinova_gen3.urdf.xacro)
set(MODEL_URDF  ${CMAKE_CURRENT_BINARY_DIR}/kinova_gen3.urdf)
add_custom_command(
  OUTPUT ${MODEL_URDF}
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/strip_ros2_control.py
          ${MODEL_XACRO} ${MODEL_URDF}
  DEPENDS ${MODEL_XACRO} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/strip_ros2_control.py
  COMMENT "Expanding kinova_gen3.urdf.xacro and stripping ros2_control")
add_custom_target(kinova_arm_urdf ALL DEPENDS ${MODEL_URDF})

install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
install(FILES ${MODEL_URDF} DESTINATION share/${PROJECT_NAME}/urdf)
install(PROGRAMS scripts/strip_ros2_control.py DESTINATION lib/${PROJECT_NAME})
ament_package()
```

> `launch/` does not exist until Task 3. Create an empty `launch/.gitkeep` now so the
> install rule does not fail.

- [ ] **Step 5: Build and verify the expansion**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_description`
Expected: build succeeds, log shows "stripped 1 ros2_control block(s)".

```bash
ssh abra 'U=/tmp/kinova-ros2-ws/install/kinova_gen3_description/share/kinova_gen3_description/urdf/kinova_gen3.urdf
  grep -c "<ros2_control" $U || true
  grep -oE "<joint name=\"joint_[0-9]\"" $U | sort -u | wc -l
  grep -c "<mimic" $U'
```
Expected: `0` ros2_control blocks, `7` arm joints, `5` mimics.

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_description/
git commit -m "feat(description): compose the Gen3 model from upstream descriptions"
```

---

### Task 2: Parity check against the current model

**Files:**
- Create: `kinova_gen3_description/test/test_model_parity.py`
- Modify: `kinova_gen3_description/CMakeLists.txt`
- Modify: `kinova_gen3_description/package.xml`

**Interfaces:**
- Consumes: the expanded `kinova_gen3.urdf` from Task 1.
- Produces: nothing later tasks depend on. This is the acceptance gate for the model
  swap.

This is the task that decides whether the generated model is usable. Gravity
compensation, cuRobo, the in-loop IK and the DLS twist solve all read these numbers, and
RViz looking correct proves nothing about any of them.

- [ ] **Step 1: Write the parity test**

`kinova_gen3_description/test/test_model_parity.py`:
```python
#!/usr/bin/env python3
"""Compare the generated model against the hand-edited one it replaces.

A difference here is a FINDING, not a tolerance to widen. The old model is what gravity
compensation, cuRobo, the in-loop IK and the DLS twist solve have always run against; if
the generated one differs, we need to know exactly where and decide deliberately. The
upstream description may well be more correct -- but that is a decision to make with the
numbers in hand.
"""
import os
import numpy as np
import pinocchio as pin
import pytest

NEW = os.environ["KINOVA_NEW_URDF"]
OLD = os.environ["KINOVA_OLD_URDF"]
# The EE frame is renamed by this change, so the comparison is old-name vs new-name.
OLD_EE, NEW_EE = "gen3_end_effector_link", "end_effector_link"
TOL = 1e-9
N_SAMPLES = 1000


def _load(path):
    model = pin.buildModelFromUrdf(path)
    return model, model.createData()


@pytest.fixture(scope="module")
def models():
    return _load(OLD), _load(NEW)


def _configs(model, n, seed=0):
    """Uniform over [-pi, pi] rather than pin.randomConfiguration: the Gen3's continuous
    joints are packed (cos, sin) in Pinocchio, so randomConfiguration would not give
    comparable q vectors across two models whose joint ORDER we are also checking."""
    rng = np.random.default_rng(seed)
    return [rng.uniform(-np.pi, np.pi, model.nq) for _ in range(n)]


def test_same_dof(models):
    (old_m, _), (new_m, _) = models
    assert old_m.nq == new_m.nq, f"nq {old_m.nq} != {new_m.nq}"
    assert old_m.nv == new_m.nv, f"nv {old_m.nv} != {new_m.nv}"


def test_forward_kinematics_matches(models):
    (old_m, old_d), (new_m, new_d) = models
    old_id, new_id = old_m.getFrameId(OLD_EE), new_m.getFrameId(NEW_EE)
    worst = 0.0
    for q in _configs(old_m, N_SAMPLES):
        pin.forwardKinematics(old_m, old_d, q); pin.updateFramePlacement(old_m, old_d, old_id)
        pin.forwardKinematics(new_m, new_d, q); pin.updateFramePlacement(new_m, new_d, new_id)
        d = np.linalg.norm(old_d.oMf[old_id].translation - new_d.oMf[new_id].translation)
        worst = max(worst, d)
    assert worst < TOL, f"EE position differs by up to {worst:.3e} m"


def test_gravity_torques_match(models):
    """The inertial parameters, made observable. A drifting arm in gravity comp is what
    a mismatch here looks like on hardware."""
    (old_m, old_d), (new_m, new_d) = models
    worst = 0.0
    for q in _configs(old_m, N_SAMPLES):
        g_old = pin.computeGeneralizedGravity(old_m, old_d, q)
        g_new = pin.computeGeneralizedGravity(new_m, new_d, q)
        worst = max(worst, float(np.max(np.abs(g_old - g_new))))
    assert worst < TOL, f"gravity torque differs by up to {worst:.3e} N*m"


def test_link_masses_match(models):
    (old_m, _), (new_m, _) = models
    old = {n: i.mass for n, i in zip(old_m.names, old_m.inertias)}
    new = {n: i.mass for n, i in zip(new_m.names, new_m.inertias)}
    total_old, total_new = sum(old.values()), sum(new.values())
    assert abs(total_old - total_new) < TOL, (
        f"total mass {total_old:.6f} vs {total_new:.6f} kg")
```

- [ ] **Step 2: Register the test**

In `CMakeLists.txt`, before `ament_package()`:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model_parity test/test_model_parity.py
    ENV
      KINOVA_NEW_URDF=${MODEL_URDF}
      KINOVA_OLD_URDF=${CMAKE_CURRENT_SOURCE_DIR}/../../kinova-gen3-driver/models/gen3_7dof_2f85.urdf)
endif()
```
and add to `package.xml`:
```xml
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>
```

> The old-model path reaches into the core checkout beside this repo, which is how the
> colcon workspace is laid out (`src/kinova-gen3-driver`, `src/kinova_gen3_ros2`). If the
> file is absent the test errors rather than silently passing — that is intended.

- [ ] **Step 3: Run it**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_description` then
```bash
ssh abra "/tmp/rtest.sh model_parity"
```

**Expected: this may FAIL, and that is a legitimate outcome.** If it does:
1. Do not widen `TOL`.
2. Record which assertion failed and by how much.
3. Stop and report. A kinematic difference means the two models are not the same robot;
   a mass difference means gravity compensation would change on hardware. Either is a
   decision for the user, not a diff to wave through.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_description/test kinova_gen3_description/CMakeLists.txt \
        kinova_gen3_description/package.xml
git commit -m "test(description): parity check the generated model against the hand-edited one"
```

---

### Task 3: Launch files

**Files:**
- Create: `kinova_gen3_description/launch/description.launch.py`
- Create: `kinova_gen3_description/launch/bringup.launch.py`
- Delete: `kinova_gen3_description/launch/.gitkeep`

**Interfaces:**
- Consumes: the installed xacro and expanded URDF from Task 1.
- Produces: `description.launch.py` (args `gripper`, `camera`, `prefix`) and
  `bringup.launch.py` (those plus `sim`, `ip`, `arbitration_mode`,
  `estop_clear_max_age_s`).

- [ ] **Step 1: Write `description.launch.py`**

```python
"""Publish /robot_description and run robot_state_publisher.

Deliberately contains nothing arm-specific and starts no driver, so a client that wants
only TF -- a planner, a perception node, RViz -- can include it without owning the arm.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper = LaunchConfiguration("gripper")
    camera = LaunchConfiguration("camera")
    prefix = LaunchConfiguration("prefix")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("kinova_gen3_description"), "urdf", "kinova_gen3.urdf.xacro"])
    robot_description = Command([
        "xacro ", xacro_file,
        " gripper:=", gripper, " camera:=", camera, " prefix:=", prefix])

    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="robotiq_2f_85",
                              description="robotiq_2f_85 | none"),
        DeclareLaunchArgument("camera", default_value="true",
                              description="include the wrist vision module"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="joint/link name prefix; empty by default"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            # value_type=str: without it the Command substitution is passed as a
            # ParameterValue of guessed type and robot_description arrives mangled.
            parameters=[{"robot_description": robot_description}],
        ),
    ])
```

> Note: `parameters=[{"robot_description": robot_description}]` relies on launch_ros
> coercing the `Command` substitution to a string. If `robot_state_publisher` reports an
> empty or malformed description, wrap it:
> `from launch_ros.parameter_descriptions import ParameterValue` and use
> `ParameterValue(robot_description, value_type=str)`.

- [ ] **Step 2: Write `bringup.launch.py`**

```python
"""Start the driver and TF together.

`urdf` defaults to this package's expanded model, so robot_state_publisher and the
driver use the SAME model by default -- which is the point of the package. Point it
elsewhere and you are on your own for keeping them consistent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("kinova_gen3_description")
    default_urdf = PathJoinSubstitution([share, "urdf", "kinova_gen3.urdf"])

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "description.launch.py"])),
        launch_arguments={
            "gripper": LaunchConfiguration("gripper"),
            "camera": LaunchConfiguration("camera"),
            "prefix": LaunchConfiguration("prefix"),
        }.items(),
    )

    sim_args = ["--sim", "--urdf", LaunchConfiguration("urdf")]
    real_args = ["--ip", LaunchConfiguration("ip"), "--urdf", LaunchConfiguration("urdf")]

    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="robotiq_2f_85"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("sim", default_value="true",
                             description="run against SimTransport instead of the arm"),
        DeclareLaunchArgument("ip", default_value="192.168.1.10",
                             description="arm IP; used only when sim:=false"),
        DeclareLaunchArgument("urdf", default_value=default_urdf),
        DeclareLaunchArgument("arbitration_mode", default_value="disabled",
                             description="enforced | disabled; read-only at runtime"),
        DeclareLaunchArgument("estop_clear_max_age_s", default_value="1.0"),
        description,
        Node(
            package="kinova_gen3_ros2", executable="kinova_gen3_node", output="screen",
            condition=IfCondition(LaunchConfiguration("sim")),
            arguments=sim_args,
            parameters=[{
                "arbitration_mode": LaunchConfiguration("arbitration_mode"),
                "estop_clear_max_age_s": LaunchConfiguration("estop_clear_max_age_s"),
            }],
        ),
        Node(
            package="kinova_gen3_ros2", executable="kinova_gen3_node", output="screen",
            condition=IfCondition(
                PythonExpression(["not ", LaunchConfiguration("sim")])),
            arguments=real_args,
            parameters=[{
                "arbitration_mode": LaunchConfiguration("arbitration_mode"),
                "estop_clear_max_age_s": LaunchConfiguration("estop_clear_max_age_s"),
            }],
        ),
    ])
```
- [ ] **Step 3: Build and launch it**

```bash
./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  timeout 20 ros2 launch kinova_gen3_description bringup.launch.py sim:=true 2>&1 | tail -20"'
```
Expected: `robot_state_publisher` reports the segments it found, and `kinova_gen3_node`
logs `kinova_gen3_node up (sim)`.

- [ ] **Step 4: Commit**

```bash
git rm kinova_gen3_description/launch/.gitkeep
git add kinova_gen3_description/launch/
git commit -m "feat(description): description and bringup launch files"
```

---

### Task 4: Prove TF actually updates

**Files:**
- Create: `kinova_gen3_description/test/test_tf_updates.py`
- Modify: `kinova_gen3_description/CMakeLists.txt`

**Interfaces:**
- Consumes: `bringup.launch.py` from Task 3.
- Produces: nothing.

The defect this package exists to fix is that TF was silently frozen. That deserves a
test, not an eyeball on RViz.

- [ ] **Step 1: Write the test**

`kinova_gen3_description/test/test_tf_updates.py`:
```python
#!/usr/bin/env python3
"""TF must actually MOVE when the arm does.

The defect this package fixes was silent: the driver published joint_1..joint_7 while
the URDF declared gen3_joint_1..gen3_joint_7, so robot_state_publisher matched nothing
and held TF at the default configuration forever. Nothing errored. A test that only
checks "a transform exists" would have passed the whole time -- so this asserts the
transform CHANGES.
"""
import math
import time

import rclpy
import pytest
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

BASE, TOOL = "base_link", "end_effector_link"
JOINTS = [f"joint_{i}" for i in range(1, 8)]


def _translation(buf, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            t = buf.lookup_transform(BASE, TOOL, rclpy.time.Time()).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            time.sleep(0.05)
    return None


def test_tf_follows_joint_states():
    rclpy.init()
    node = Node("tf_update_test")
    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(JointState, "/joint_states", 10)

    def publish(angle):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [0.0, angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)

    try:
        for _ in range(20):
            publish(0.0); rclpy.spin_once(node, timeout_sec=0.05)
        first = _translation(buf)
        assert first is not None, f"no transform {BASE} -> {TOOL}; RSP is not running"

        for _ in range(20):
            publish(math.pi / 4); rclpy.spin_once(node, timeout_sec=0.05)
        second = _translation(buf)
        assert second is not None

        moved = max(abs(a - b) for a, b in zip(first, second))
        assert moved > 0.01, (
            f"TF did not move when joint_2 went to pi/4 (max delta {moved:.6f} m). "
            "This is the frozen-TF defect: joint names in /joint_states do not match "
            "the URDF.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Register it**

In `CMakeLists.txt`'s `BUILD_TESTING` block:
```cmake
  ament_add_pytest_test(tf_updates test/test_tf_updates.py)
```
and add to `package.xml`:
```xml
  <test_depend>tf2_ros</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>sensor_msgs</test_depend>
```

> This test needs `robot_state_publisher` running. Run it against a live
> `description.launch.py` rather than as a bare unit test — see Step 3.

- [ ] **Step 3: Run it against a live description**

```bash
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  ros2 launch kinova_gen3_description description.launch.py > /tmp/rsp.log 2>&1 &
  sleep 6
  python3 /tmp/kinova-ros2-ws/src/kinova_gen3_ros2/kinova_gen3_description/test/test_tf_updates.py
  pkill -f robot_state_publisher"'
```
Expected: the assertion passes — TF moves when `joint_2` does.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_description/test/test_tf_updates.py kinova_gen3_description/CMakeLists.txt \
        kinova_gen3_description/package.xml
git commit -m "test(description): TF must move when the arm does"
```

---

### Task 5: Point the driver at the shared model, and document

**Files:**
- Modify: `README.md`
- Modify: `kinova_gen3_ros2/src/ros2_backend.cpp` (only if Task 2 confirms parity)

**Interfaces:**
- Consumes: everything above.
- Produces: nothing.

- [ ] **Step 1: Document the package in `README.md`**

Add a "Robot model and TF" section covering: the package exists and why (the frozen-TF
defect and the unregenerable model); `ros2 launch kinova_gen3_description
bringup.launch.py sim:=true`; the xacro args; that the model is composed from upstream
rather than vendored; and that `rviz2` needs `/robot_description` from the description
launch.

State plainly that **the driver publishes `joint_1..joint_7` and the model must agree** —
that is the invariant the whole package exists to hold.

- [ ] **Step 2: Note the gripper joint is not yet published**

In the same section, record that `robotiq_85_left_knuckle_joint` is **not** in
`/joint_states` yet, so the gripper renders at its default opening. Publishing it needs
core to supply the gripper position through `ArmState`, and it is one joint plus five
mimics that `robot_state_publisher` derives — see the spec's "Gripper joint state".

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: the description package, TF, and launch"
```

---

## Deliberate scope calls

- **The driver's `--urdf` default is not changed.** It still resolves relative to the cwd,
  as the container and `abra_e2e_sim.sh` expect. `bringup.launch.py` passes this package's
  model explicitly instead, so launching gets consistency without breaking the two
  existing entry points.
- **Core's `Dynamics` `ee_frame` default is NOT changed here.** It is
  `gen3_end_effector_link`, which does not exist in the generated model — so anything
  using the generated model must pass `end_effector_link` explicitly. Changing the default
  is a core PR, gated on Task 2's parity result, and doing it before we know the models
  agree would be changing two things at once.
- **No RViz config**, per the approved scope.
