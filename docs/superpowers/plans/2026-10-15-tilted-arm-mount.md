# Tilted Arm Mount (kinova-gen3-ros2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the arm's model once, at launch, from the robot's `mount.yaml` (root `arm_mount`, every name `arm_`-prefixed, plus `arm_tool_frame`), publish it latched on `/arm/robot_description`, and make the driver node build its dynamics, its `/joint_states` names and its Cartesian frame check from that one model.

**Architecture:** `kinova_gen3_description` loses its build-time expansion. A single Python module, `kinova_gen3_render.py`, runs xacro with the mount file and strips `<ros2_control>`. `description.launch.py` calls it and runs `robot_state_publisher` with `robot_description` remapped to `/arm/robot_description`. The bench uses the same module through a CLI. `kinova_gen3_node` drops `--urdf`. Before the RT loop exists it waits for the latched topic, builds `Dynamics` from the XML through the driver's new `UrdfXml` constructor, which reduces the 13-DOF model to the 7 arm joints, and passes `dyn.joint_names()` plus the gripper joint to `Ros2Backend`. `GoToEEPose` accepts only `arm_mount`.

**Tech Stack:** ROS 2 Humble, xacro 2.1, robot_state_publisher, launch_testing, ament_cmake_pytest, gtest, `urdf` (urdfdom), Pinocchio 3.9 (pip/cmeel), kinova-gen3-driver (`kinova_lowlevel`), Docker (rammp-base 1.1.0-jp6, arm64).

**Spec:** rammp-deployments: `docs/superpowers/specs/2026-10-15-tilted-arm-mount-design.md`. Its "kinova-gen3-ros2" section is this plan's scope. So are the "What changes for consumers" rows and "Rollout" steps that land in this repo.

## Global Constraints

- URDF root link: `arm_mount`. Every arm link and joint carries the `arm_` prefix: `arm_base_link` … `arm_bracelet_link`, `arm_end_effector_link`, `arm_joint_1` … `arm_joint_7`. Gripper joint: `arm_robotiq_85_left_knuckle_joint`.
- New fixed link `arm_tool_frame`, child of `arm_end_effector_link`. It uses the same transform as cuRobo v0.7.8's bundled `kinova_gen3_7dof.urdf` `tool_frame`, which is `xyz="0 0 0.120" rpy="0 0 0"` (verbatim; source quoted in Task 1).
- Model topic: `/arm/robot_description`, `std_msgs/String`, RELIABLE + TRANSIENT_LOCAL. It is robot_state_publisher's `robot_description`, remapped. `/tf` and `/joint_states` stay global.
- Mount file: YAML `robot: <name>` and `rpy_deg: [r, p, y]`, in degrees, giving the orientation of `arm_base_link` in `arm_mount`. Unknown keys are errors. Containers see it at `/config/robot/mount.yaml`.
- `description.launch.py` takes the REQUIRED launch argument `mount_file`, with no default. The xacro takes the required argument `mount_file`, reads it with `xacro.load_yaml` and converts it with `radians()`. At startup the description logs `embodiment: <robot> (mount rpy_deg [r, p, y])` before it starts anything.
- No model is rendered at build time. No `kinova_gen3.urdf` or `kinova_gen3_7dof.urdf` exists in any install space or image. The `<ros2_control>` block is still stripped.
- Bench render: `kinova_gen3_render.py --mount <file> --out <urdf>`.
- Driver node: `--urdf` is removed. The node reads `/arm/robot_description` once, before the RT loop, and on timeout it fails with an error that names `arm_state_publisher`. `--ee-frame` defaults to `arm_end_effector_link` and is always passed to `Dynamics` explicitly. The library keeps its own default, `gen3_end_effector_link`, for its test model.
- `/joint_states` names are `dyn.joint_names()` followed by `arm_robotiq_85_left_knuckle_joint`. The node checks that this joint is present in the URDF at startup and throws otherwise. No joint name is hard-coded in `ros2_backend.cpp`.
- `GoToEEPose` requires `frame_id == "arm_mount"`. For `base_link` the rejection text is exactly: `frame_id 'base_link' is no longer accepted: Cartesian goals are in 'arm_mount' (gravity-aligned); arm_base_link is tilted 30° on flatbot and gen_1_5`
- Driver API consumed (kinova-gen3-driver, parallel plan): `struct kinova::UrdfXml { std::string xml; };` in `kinova_lowlevel/dynamics.h`, plus `Dynamics(const UrdfXml&, const std::string& ee_frame)` and `std::vector<std::string> Dynamics::joint_names() const`, which returns the 7 arm joints in chain order. Model reduction happens inside `Dynamics`.
- Out of scope (do not touch): IMU checks, `SetGravityVector`, camera spheres, cuRobo, per-robot worlds, `.msg`/`.action` edits, sheppy, `rammp-deployments`, and the planner fake's `joint_names` in `test/fake_curobo_server.h` (the driver ignores trajectory joint names).
- Arms are powered OFF on abra and snorlax. All verification is sim or offline. Do not push branches.

## Test environment (abra, aarch64, sim/offline only)

Tests run natively on **abra** (dojo's Jetson). Use an isolated workspace, `~/mount_ws`, so that nothing in `~/ros2_ws` shadows or gets rebuilt. Two quirks on abra are load-bearing:

- Its login shell is zsh. Every remote command therefore runs under `bash -c` after sourcing `/opt/ros/humble/setup.bash`.
- `~/.local` holds a stale cmeel prefix (pin 2.7.0). Its `pinocchioTargets.cmake` points at a `libpinocchio.so` that does not exist, and `$(python3 -m cmeel cmake)` lists that prefix first. Point CMake at `/usr/local/lib/python3.10/dist-packages/cmeel.prefix` only, the same prefix the Dockerfile uses.

One-time setup (from the Mac). The interfaces clone must carry `EeState.wrench`. `dev` of this repo already writes it (`ros2_backend.cpp:245-250`), but when this plan was written that field existed only on rammp-interfaces-ros2 `feat/ee-state-wrench`, not on its `dev`. Check `dev` first with `git -C ~/mount_ws/src/rammp-interfaces-ros2 grep -c "Wrench wrench" -- rammp_arm_interfaces/msg/EeState.msg` and use `dev` if that prints `1`:

```bash
ssh abra 'mkdir -p ~/mount_ws/src && cd ~/mount_ws && \
  git clone -q --branch feat/ee-state-wrench https://github.com/rammp-org/rammp-interfaces-ros2 src/rammp-interfaces-ros2 && \
  git clone -q --branch v1.1.0 https://github.com/rammp-org/RAMMP-CuRobo src/RAMMP-CuRobo'
ssh abra 'cat > ~/mount_ws/run.sh' <<'EOF'
#!/usr/bin/env bash
# run.sh <package> <ctest regex>: build <package> and its deps, run the matching tests.
set -eo pipefail
cd ~/mount_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix:${CMAKE_PREFIX_PATH:-}
export ROS_DOMAIN_ID=79   # off the fleet's domain 0
colcon build --packages-up-to "$1" --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DKINOVA_ENABLE_KORTEX=OFF
colcon test --packages-select "$1" --ctest-args -R "$2"
colcon test-result --verbose --test-result-base "build/$1"
EOF
```

Sync before every remote run. This repo's worktree goes to `src/kinova_gen3_ros2`, the same layout the README's native build uses, so that `../../kinova-gen3-driver/models` resolves. The driver's worktree goes to `src/kinova-gen3-driver`:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-driver-mount/ abra:mount_ws/src/kinova-gen3-driver/
```

**Prerequisite for Tasks 3–6:** `/Users/swapnil/atdev/kinova-gen3-driver-mount` contains the driver's `UrdfXml` constructor and `Dynamics::joint_names()`. Check:
`grep -n "struct UrdfXml\|joint_names" /Users/swapnil/atdev/kinova-gen3-driver-mount/include/kinova_lowlevel/dynamics.h`
Expected: one line for each. If either is missing, stop and ask. Do not stub the driver API. Tasks 1–2 build `kinova_gen3_description` only, and that package does not depend on the driver.

`colcon test` on a package that does not compile prints `Has this package been built before?` and reports 0 tests. Treat that as a failure, not a pass.

## Review Focus

1. **Launch order:** the driver may start before `arm_state_publisher` (sheppy starts containers concurrently) or after it. A reasonable person expects both orders to work as long as the model arrives within the timeout. Pinned by `RobotDescriptionTest.ALatchedDescriptionPublishedEarlierArrives` and `…PublishedAfterTheDriverStartsWaitingArrives` (Task 3).
1. **Bad or missing mount file:** a typo'd key, NaN, a quoted number, two angles, a list instead of a mapping, or a path that does not exist. Docker creates an empty directory at a missing bind-mount source, so `/config/robot/mount.yaml` is then simply absent. Each case must fail loudly and name the file, never render a level arm. Pinned by `test_a_bad_mount_file_is_refused_by_name` and `test_a_missing_mount_file_is_an_error` (Task 1).
1. **Wrong model on the topic:** an unprefixed or old URDF ends up on `/arm/robot_description`. The node must refuse to start and name the missing joint, rather than publish names that freeze TF. Pinned by `JointStateNames.AModelWithoutTheGripperJointIsRefusedByName` (Task 3). `Dynamics` itself throws on a missing `arm_end_effector_link`.
1. **Default-constructed GoToEEPose goal:** an empty `frame_id`, which is what a client that forgets the header sends. It must be rejected, with the reason naming `arm_mount`. Pinned by `MessageMapping.EePoseFrameMustBeArmMount` (Task 5).
1. **SIGTERM while waiting for the model:** `docker stop` during startup must exit cleanly, without a misleading "no model" FATAL. This is handled in `main()` by checking `rclcpp::ok()` before logging (Task 4). It is checked by hand in Task 6 Step 6, because `main()` has no unit test.

______________________________________________________________________

## File Structure

`kinova_gen3_description/`

- `urdf/kinova_gen3.urdf.xacro`: modify. Adds the `arm_mount` root, the fixed `arm_` prefix, the required `mount_file` and `arm_tool_frame`.
- `scripts/kinova_gen3_render.py`: create. `load_mount`, `embodiment_line`, `render`, and the bench CLI. This is the only renderer.
- `scripts/expand_model.py`: delete. Build-time expansion is gone.
- `launch/description.launch.py`: rewrite. Renders at launch, logs the embodiment line, remaps the topic.
- `launch/bringup.launch.py`: modify. Takes `mount_file`, no `--urdf`, new `ee_frame` default.
- `CMakeLists.txt`, `package.xml`: modify. No build-time render, Python module install, three tests.
- `test/mounts/level.yaml`, `test/mounts/tilted.yaml`: create. Test fixtures only.
- `test/test_model_parity.py`: rewrite. Covers parity with the old model, the mount, the tool frame and mount validation.
- `test/test_description_log.py`, `test/test_description_launch.py`: create.
- `test/test_tf_updates.py`: modify (manual live script). Uses the new names.

`kinova_gen3_ros2/`

- `include/kinova_gen3_ros2/robot_description.h`, `src/robot_description.cpp`: create. Waits for the latched model and derives the `/joint_states` names.
- `include/kinova_gen3_ros2/ros2_backend.h`, `src/ros2_backend.cpp`: modify. Joint names are injected.
- `include/kinova_gen3_ros2/message_mapping.h`, `src/message_mapping.cpp`: modify. Adds `ee_pose_frame_rejection`.
- `include/kinova_gen3_ros2/goto_ee_pose_server.h`: modify. Uses it.
- `src/bringup_node.cpp`: modify. Reads the topic, builds `UrdfXml` Dynamics, wires the names.
- `CMakeLists.txt`, `package.xml`: modify. New library and test, `urdf` dependency.
- `test/robot_description_test.cpp`: create. Also modify `test/ros2_backend_test.cpp`, `test/gripper_server_test.cpp`, `test/message_mapping_test.cpp` and `test/goto_ee_pose_integration_test.cpp`.
- Client scripts and conformance: `test/send_goto_pose*.py`, `test/scenario_carry_level.py`, `test/sweep_constraints.py`, `test/probe_reachable.py`, `test/conformance/checks_state.py`, `test/conformance/checks_gripper.py`, `test/conformance/README.md`.

Repo root: `Dockerfile`, `Makefile`, `scripts/smoke.sh`, `deploy/compose.yaml`, new `rammp-alternative.description.yaml`, `kinova_gen3.repos`, `README.md`, `docs/*.md(x)` and `CHANGELOG.md`.

______________________________________________________________________

### Task 1: The model: `arm_mount` root, `arm_` names, mount file, `arm_tool_frame`, rendered by one module

**Files:**

- Modify: `kinova_gen3_description/urdf/kinova_gen3.urdf.xacro` (whole file, 48 lines)
- Create: `kinova_gen3_description/scripts/kinova_gen3_render.py`
- Delete: `kinova_gen3_description/scripts/expand_model.py`
- Modify: `kinova_gen3_description/CMakeLists.txt` (whole file, 49 lines)
- Modify: `kinova_gen3_description/package.xml` (whole file)
- Create: `kinova_gen3_description/test/mounts/level.yaml`, `kinova_gen3_description/test/mounts/tilted.yaml`
- Test: `kinova_gen3_description/test/test_model_parity.py` (rewrite)

**Interfaces:**

- Consumes: nothing from other tasks.

- Produces (Python module `kinova_gen3_render`, importable once installed and in tests via `PYTHONPATH=<src>/scripts`):

  - `load_mount(path: str) -> dict`: raises `ValueError` whose message starts with `path`.
  - `embodiment_line(mount: dict) -> str`: returns `"embodiment: <robot> (mount rpy_deg [r, p, y])"` with floats.
  - `render(mount_file: str) -> tuple[str, dict]`: returns `(urdf_xml, mount)`, with `<ros2_control>` stripped.
  - CLI: `kinova_gen3_render.py --mount <file> --out <urdf>`. `ros2 run kinova_gen3_description kinova_gen3_render.py …` works after install.
  - Test fixtures: `test/mounts/level.yaml` (`robot: test_level`, `[0.0, 0.0, 0.0]`) and `test/mounts/tilted.yaml` (`robot: test_tilted`, `[0.0, 30.0, 0.0]`).
  - URDF names: root `arm_mount`, `arm_base_joint` (fixed, `arm_mount` → `arm_base_link`, mount rpy), `arm_tool_frame_joint` → `arm_tool_frame`, and `arm_robotiq_85_left_knuckle_joint`.

- [ ] **Step 1: Write the fixtures and the failing test**

`kinova_gen3_description/test/mounts/level.yaml`:

```yaml
# Test fixture: a level arm (dojo-like). Not a default for anything.
robot: test_level
rpy_deg: [0.0, 0.0, 0.0]
```

`kinova_gen3_description/test/mounts/tilted.yaml`:

```yaml
# Test fixture: new_adapter_plate_v3's 30 deg forward lean (flatbot-like).
robot: test_tilted
rpy_deg: [0.0, 30.0, 0.0]
```

Replace `kinova_gen3_description/test/test_model_parity.py` entirely:

```python
#!/usr/bin/env python3
"""Compare the rendered model against the hand-edited one it replaced, and pin the mount.

Gravity compensation, cuRobo, the in-loop IK and the DLS twist solve all read these
numbers, and RViz looking correct proves nothing about any of them. So the model is
gated here rather than by eye.

Two differences from the hand-edited model are KNOWN and deliberate; everything else
about the arm must match exactly.

  1. The gripper articulates (13 DOF). The driver reduces it to the seven arm joints
     itself, locking the rest at q = 0 -- reproduced here with buildReducedModel.

  2. 0.436 kg is missing at the wrist -- a RealSense D415 (0.072 kg) and a 0.5 kg
     custom camera mount that the c3pzero author added and upstream cannot know about.
     Deferred by decision, and PINNED by test_known_wrist_mass_delta below so it cannot
     drift or be forgotten: add the camera back and that test fails, telling you to.
"""

import os

import numpy as np
import pinocchio as pin
import pytest
from kinova_gen3_render import embodiment_line, load_mount, render

HERE = os.path.dirname(os.path.abspath(__file__))
LEVEL = os.path.join(HERE, "mounts", "level.yaml")
TILTED = os.path.join(HERE, "mounts", "tilted.yaml")
OLD = os.environ["KINOVA_OLD_URDF"]
OLD_EE, NEW_EE = "gen3_end_effector_link", "arm_end_effector_link"
OLD_ARM = [f"gen3_joint_{i}" for i in range(1, 8)]
ARM = [f"arm_joint_{i}" for i in range(1, 8)]
GRIPPER_JOINT = "arm_robotiq_85_left_knuckle_joint"
TOL = 1e-9
KNOWN_WRIST_MASS_DELTA_KG = 0.436
N_SAMPLES = 500

# cuRobo v0.7.8, src/curobo/content/assets/robot/kinova/kinova_gen3_7dof.urdf, verbatim:
#   <joint name="end_effector" type="fixed">
#     <origin rpy="3.14159265358979 1.09937075168372E-32 0" xyz="0 0 -0.0615250000000001"/>
#     <parent link="bracelet_link"/> <child link="end_effector_link"/>
#   <joint name="tool_frame_joint" type="fixed">
#     <origin rpy="0 0 0" xyz="0 0 0.120"/>
#     <parent link="end_effector_link"/> <child link="tool_frame"/>
CUROBO_BRACELET_TO_EE = pin.SE3(
    pin.rpy.rpyToMatrix(3.14159265358979, 1.09937075168372e-32, 0.0),
    np.array([0.0, 0.0, -0.0615250000000001]),
)
CUROBO_EE_TO_TOOL = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.120]))


def _model(mount_file):
    xml, _ = render(mount_file)
    return pin.buildModelFromXML(xml)


def _reduce_to_arm(model):
    """What the driver does: lock every joint that is not an arm joint at q = 0."""
    lock = [model.getJointId(n) for n in model.names[1:] if n not in ARM]
    return pin.buildReducedModel(model, lock, pin.neutral(model))


def _set_arm(model, angles, names):
    """Write the seven arm angles into q BY JOINT NAME (continuous joints are packed
    (cos, sin), so slicing would compare different joints and report agreement)."""
    q = pin.neutral(model)
    for name, a in zip(names, angles):
        j = model.joints[model.getJointId(name)]
        if j.nq == 2:
            q[j.idx_q], q[j.idx_q + 1] = np.cos(a), np.sin(a)
        else:
            q[j.idx_q] = a
    return q


def _placement(model, q, frame):
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[model.getFrameId(frame)]


def _arm_configs(n, seed=0):
    rng = np.random.default_rng(seed)
    return [rng.uniform(-np.pi, np.pi, len(ARM)) for _ in range(n)]


@pytest.fixture(scope="module")
def level():
    return _model(LEVEL)


@pytest.fixture(scope="module")
def tilted():
    return _model(TILTED)


@pytest.fixture(scope="module")
def old():
    return pin.buildModelFromUrdf(OLD)


def test_root_is_arm_mount_and_everything_is_prefixed(level):
    assert level.frames[1].name == "arm_mount", level.frames[1].name
    unprefixed = [f.name for f in level.frames[1:] if not f.name.startswith("arm_")]
    assert unprefixed == [], f"frames without the arm_ prefix: {unprefixed}"


def test_articulated_with_the_gripper_joint(level):
    assert level.nv == 13, f"expected the 13-DOF articulated model, got nv={level.nv}"
    assert level.existJointName(GRIPPER_JOINT)


def test_no_ros2_control_block():
    xml, _ = render(LEVEL)
    assert "ros2_control" not in xml


def test_reduced_model_is_seven_dof_in_chain_order(level):
    seven = _reduce_to_arm(level)
    assert seven.nv == 7
    assert list(seven.names[1:]) == ARM


def test_arm_link_masses_are_identical(level, old):
    for o, n in zip(OLD_ARM[:-1], ARM[:-1]):  # joint_7 carries the wrist delta
        mo = old.inertias[old.getJointId(o)].mass
        mn = level.inertias[level.getJointId(n)].mass
        assert abs(mo - mn) < TOL, f"{n} body mass {mo:.6f} != {mn:.6f} kg"


def test_level_mount_forward_kinematics_matches_old_model(level, old):
    """Identity mount: arm_mount IS arm_base_link, so every number is unchanged."""
    seven = _reduce_to_arm(level)
    worst = 0.0
    for a in _arm_configs(N_SAMPLES):
        po = _placement(old, _set_arm(old, a, OLD_ARM), OLD_EE)
        pn = _placement(seven, _set_arm(seven, a, ARM), NEW_EE)
        worst = max(worst, float(np.linalg.norm(po.translation - pn.translation)))
    assert worst < TOL, f"EE position differs by up to {worst:.6e} m"


def test_known_wrist_mass_delta(level, old):
    delta = sum(i.mass for i in old.inertias) - sum(i.mass for i in level.inertias)
    assert abs(delta - KNOWN_WRIST_MASS_DELTA_KG) < 1e-3, (
        f"total-mass delta is {delta:.6f} kg, expected {KNOWN_WRIST_MASS_DELTA_KG:.6f} "
        "(the omitted wrist camera + mount). If you added the camera back, set "
        "KNOWN_WRIST_MASS_DELTA_KG to 0.0; if something else moved, find out what."
    )


def test_tool_frame_matches_curobo(level):
    """cuRobo positions every PlanToPose goal at tool_frame; ours must be the same
    point, measured from the bracelet (the last moving body), so a difference in
    either fixed joint shows up."""
    q = pin.neutral(level)
    ours = _placement(level, q, "arm_bracelet_link").inverse() * _placement(
        level, q, "arm_tool_frame"
    )
    theirs = CUROBO_BRACELET_TO_EE * CUROBO_EE_TO_TOOL
    assert np.allclose(ours.translation, theirs.translation, atol=1e-9)
    assert np.allclose(ours.rotation, theirs.rotation, atol=1e-9)


def test_tilted_mount_leans_arm_base_forward(tilted):
    """+30 deg about Y: arm_base_link's +Z points along (0.5, 0, 0.866) in arm_mount."""
    z = _placement(tilted, pin.neutral(tilted), "arm_base_link").rotation[:, 2]
    assert np.allclose(z, [0.5, 0.0, np.sqrt(3) / 2], atol=1e-12), z


def test_tilted_forward_kinematics_is_the_level_one_rotated(level, tilted):
    """The mount only rotates: FK in arm_mount = R_y(30 deg) * FK of the level arm."""
    R = pin.rpy.rpyToMatrix(0.0, np.radians(30.0), 0.0)
    for a in _arm_configs(50):
        pl = _placement(level, _set_arm(level, a, ARM), "arm_tool_frame")
        pt = _placement(tilted, _set_arm(tilted, a, ARM), "arm_tool_frame")
        assert np.allclose(pt.translation, R @ pl.translation, atol=1e-12)
        assert np.allclose(pt.rotation, R @ pl.rotation, atol=1e-12)


def test_embodiment_line_names_the_robot_and_the_rotation():
    assert (
        embodiment_line(load_mount(TILTED))
        == "embodiment: test_tilted (mount rpy_deg [0.0, 30.0, 0.0])"
    )


@pytest.mark.parametrize(
    "body, why",
    [
        ("robot: x\n", "missing keys ['rpy_deg']"),
        ("robot: x\nrpy_deg: [0, 30, 0]\nxyz: [0, 0, 1]\n", "unknown keys ['xyz']"),
        ("robot: x\nrpy_deg: [0, 30]\n", "rpy_deg must be three finite numbers"),
        ("robot: x\nrpy_deg: [0, .nan, 0]\n", "rpy_deg must be three finite numbers"),
        ("robot: x\nrpy_deg: [0, '30', 0]\n", "rpy_deg must be three finite numbers"),
        ("- 1\n- 2\n", "expected a mapping"),
    ],
)
def test_a_bad_mount_file_is_refused_by_name(tmp_path, body, why):
    p = tmp_path / "mount.yaml"
    p.write_text(body)
    with pytest.raises(ValueError, match=str(p)) as e:
        render(str(p))
    assert why in str(e.value)


def test_a_missing_mount_file_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        render(str(tmp_path / "nope.yaml"))
```

Replace `kinova_gen3_description/CMakeLists.txt` entirely. Its tests find the module from the source `scripts/` directory, so this CMake is needed for the test to run at all:

```cmake
cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_description)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The model is rendered at LAUNCH, from the robot's mount file, by
# kinova_gen3_render.py -- never at build time. A build-time render baked in one
# mount (level) and left a flat URDF in the image for anything to pick up by
# mistake; on a tilted arm that silently applies gravity 30 degrees off.
install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
# Importable (description.launch.py does `from kinova_gen3_render import ...`)...
ament_python_install_module(scripts/kinova_gen3_render.py)
# ...and runnable, for the bench: the driver's standalone apps take --urdf.
install(PROGRAMS scripts/kinova_gen3_render.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # PYTEST_DISABLE_PLUGIN_AUTOLOAD: a stale anyio plugin in ~/.local on the Jetson
  # breaks pytest's entrypoint autoload outright (ModuleNotFoundError: _pytest.scope).
  # APPEND_ENV, not ENV, for the paths: the tests need ROS's own python path (xacro,
  # ament_index_python) as well as pinocchio's, and this package's install prefix so
  # get_package_share_directory finds the xacro the render reads.
  set(_test_env
    APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:/usr/local/lib/python3.10/dist-packages
      AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    ENV
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(model_parity test/test_model_parity.py
    ${_test_env}
    KINOVA_OLD_URDF=${CMAKE_CURRENT_SOURCE_DIR}/../../kinova-gen3-driver/models/gen3_7dof_2f85.urdf)
endif()

ament_package()
```

(`KINOVA_OLD_URDF=…` follows `ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` from `_test_env`, so it lands in the same `ENV` list.)

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_description model_parity'
```

Expected: FAIL. Either the build fails because CMake cannot find `scripts/kinova_gen3_render.py` for `ament_python_install_module`, or, if that file has been created first, collection fails with `ModuleNotFoundError: No module named 'kinova_gen3_render'`.

- [ ] **Step 3: Write the render module**

Create `kinova_gen3_description/scripts/kinova_gen3_render.py` (mode 755):

```python
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
```

```bash
chmod 755 kinova_gen3_description/scripts/kinova_gen3_render.py
git rm kinova_gen3_description/scripts/expand_model.py
```

- [ ] **Step 4: Write the xacro**

Replace `kinova_gen3_description/urdf/kinova_gen3.urdf.xacro` entirely. `xacro:arg` cannot be declared without a default; on xacro 2.1.1 that is `error: xacro:arg: missing attribute 'default'`. "Required" therefore means an empty default plus `xacro.fatal`. Both were verified on abra.

```xml
<?xml version="1.0"?>
<!--
  The Gen3 7-DOF, composed from upstream descriptions rather than hand-edited.

  NOT built on kortex_description's load_robot/load_gripper: that shim passes
  isaac_joint_commands / isaac_joint_states down to robotiq_description's
  robotiq_gripper macro, which accepts neither (kortex_description 0.2.3 against the
  robotiq_description in Humble). Composing load_arm and robotiq_gripper directly
  sidesteps the version skew and costs nothing; the shim adds no geometry.

  Every link and joint carries the arm_ prefix (arm_joint_1 .. arm_joint_7,
  arm_base_link, ...): the chair owns base_link (REP-105). The root is arm_mount,
  gravity-aligned on level ground and sharing arm_base_link's origin; the only thing
  between them is the mount rotation read from mount_file.
-->
<robot name="kinova_gen3" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:arg name="gripper" default="robotiq_2f_85"/>   <!-- robotiq_2f_85 | none -->
  <xacro:arg name="camera"  default="true"/>            <!-- wrist vision module -->
  <!-- REQUIRED. xacro:arg cannot be declared without a default, so the empty default
       is caught below rather than silently rendering a level arm. -->
  <xacro:arg name="mount_file" default=""/>
  <xacro:property name="prefix" value="arm_"/>

  <xacro:if value="${not '$(arg mount_file)'}">
    ${xacro.fatal('kinova_gen3.urdf.xacro: mount_file is required (a mount.yaml with robot and rpy_deg)')}
  </xacro:if>
  <!-- rpy_deg: orientation of arm_base_link in arm_mount, in degrees. -->
  <xacro:property name="mount" value="${xacro.load_yaml('$(arg mount_file)')}"/>
  <xacro:property name="mount_rpy" value="${mount['rpy_deg']}"/>

  <link name="arm_mount"/>

  <xacro:include filename="$(find kortex_description)/arms/gen3/7dof/urdf/gen3_macro.xacro"/>
  <xacro:include filename="$(find robotiq_description)/urdf/robotiq_2f_85_macro.urdf.xacro"/>

  <!-- load_arm REQUIRES connection parameters because it emits a ros2_control block we
       do not want and cannot opt out of. They are dummies: nothing here connects to an
       arm, and the block is stripped when the model is rendered (kinova_gen3_render.py
       says why that matters). -->
  <xacro:load_arm parent="arm_mount" dof="7" vision="$(arg camera)" prefix="${prefix}"
      robot_ip="0.0.0.0" username="admin" password="admin"
      port="10000" port_realtime="10001"
      session_inactivity_timeout_ms="6000" connection_inactivity_timeout_ms="2000"
      gripper_joint_name="${prefix}robotiq_85_left_knuckle_joint"
      use_internal_bus_gripper_comm="false">
    <origin xyz="0 0 0"
            rpy="${radians(mount_rpy[0])} ${radians(mount_rpy[1])} ${radians(mount_rpy[2])}"/>
  </xacro:load_arm>

  <!-- cuRobo positions every PlanToPose goal at tool_frame. Same transform as
       tool_frame_joint in cuRobo v0.7.8's bundled kinova_gen3_7dof.urdf
       (src/curobo/content/assets/robot/kinova/kinova_gen3_7dof.urdf), whose
       end_effector_link sits where upstream's does. Upstream kortex_description has
       no tool frame. test_tool_frame_matches_curobo pins it. -->
  <link name="${prefix}tool_frame"/>
  <joint name="${prefix}tool_frame_joint" type="fixed">
    <origin xyz="0 0 0.120" rpy="0 0 0"/>
    <parent link="${prefix}end_effector_link"/>
    <child link="${prefix}tool_frame"/>
  </joint>

  <!-- include_ros2_control=false: the gripper macro DOES offer the opt-out the arm
       macro lacks, so this half emits geometry only. -->
  <xacro:if value="${'$(arg gripper)' == 'robotiq_2f_85'}">
    <xacro:robotiq_gripper name="RobotiqGripper" prefix="${prefix}"
        parent="${prefix}end_effector_link" include_ros2_control="false">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:robotiq_gripper>
  </xacro:if>

</robot>
```

Replace `kinova_gen3_description/package.xml` entirely. xacro and the upstream descriptions are now needed only at run time. `ament_cmake_python` installs the module:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>kinova_gen3_description</name>
  <version>1.0.0</version>
  <description>Kinova Gen3 7-DOF robot model, composed from upstream descriptions.</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <!-- Everything below is a RUN-time dependency: the model is rendered at launch from
       the robot's mount file, never at build time. -->
  <exec_depend>xacro</exec_depend>
  <exec_depend>kortex_description</exec_depend>
  <exec_depend>robotiq_description</exec_depend>
  <exec_depend>realsense2_description</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>std_msgs</test_depend>
  <test_depend>tf2_ros_py</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 5: Run the test to verify it passes**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_description model_parity'
```

Expected: `18 passed` in the pytest output, then `Summary: … 0 errors, 0 failures`. This exact file and xacro passed 18/18 under colcon test against abra's `ros-humble-kortex-description 0.2.3` and pin 3.9.0 when this plan was written.

Then check the bench CLI and that no URDF is installed:

```bash
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && \
  ros2 run kinova_gen3_description kinova_gen3_render.py \
    --mount ~/mount_ws/src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/tilted.yaml --out /tmp/arm.urdf && \
  find ~/mount_ws/install/kinova_gen3_description -name \"*.urdf\" | wc -l'"
```

Expected: `/tmp/arm.urdf: embodiment: test_tilted (mount rpy_deg [0.0, 30.0, 0.0])`, then `0`.

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_description/urdf/kinova_gen3.urdf.xacro \
  kinova_gen3_description/scripts/kinova_gen3_render.py \
  kinova_gen3_description/CMakeLists.txt kinova_gen3_description/package.xml \
  kinova_gen3_description/test/mounts kinova_gen3_description/test/test_model_parity.py
git commit -m "feat(description): arm_mount root, arm_ names, mount file, arm_tool_frame

Rendered by kinova_gen3_render.py only; build-time expansion is removed.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

(`git rm` in Step 3 already staged the deletion of `expand_model.py`.)

______________________________________________________________________

### Task 2: `description.launch.py` renders at launch and publishes `/arm/robot_description`

**Files:**

- Modify: `kinova_gen3_description/launch/description.launch.py` (whole file, 60 lines)
- Modify: `kinova_gen3_description/CMakeLists.txt` (the `if(BUILD_TESTING)` block from Task 1)
- Create: `kinova_gen3_description/test/test_description_log.py`, `kinova_gen3_description/test/test_description_launch.py`
- Modify: `kinova_gen3_description/test/test_tf_updates.py:24, 37-38, 82-85, 94-95` (manual live script)

**Interfaces:**

- Consumes: `kinova_gen3_render.render`, `embodiment_line`, and `test/mounts/tilted.yaml` (Task 1).

- Produces:

  - Launch argument `mount_file`. It is required, and omitting it fails with `Included launch description missing required argument 'mount_file'` (or `Required launch argument "mount_file" … was not provided` at top level).
  - Topic `/arm/robot_description`, latched. Nothing is published on `/robot_description`.
  - A module-level function `_rsp(context) -> list` in `description.launch.py`. The test loads it by path. Its first element is a `LogInfo` carrying the embodiment line, and its second is the RSP `Node`.

- [ ] **Step 1: Write the failing tests**

`kinova_gen3_description/test/test_description_log.py`:

```python
"""description.launch.py logs the embodiment line BEFORE it starts anything, so the
active robot is the first thing in the log. (LogInfo goes to launch's own logger, which
launch_testing's proc_output cannot see -- hence this offline check.)"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import LogInfo
from launch_ros.actions import Node

TILTED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mounts", "tilted.yaml")


def _launch_module():
    path = os.path.join(
        get_package_share_directory("kinova_gen3_description"),
        "launch",
        "description.launch.py",
    )
    spec = importlib.util.spec_from_file_location("description_launch", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_embodiment_line_comes_first():
    ctx = LaunchContext()
    ctx.launch_configurations["mount_file"] = TILTED
    actions = _launch_module()._rsp(ctx)
    assert isinstance(actions[0], LogInfo)
    assert "".join(s.perform(ctx) for s in actions[0].msg) == (
        "embodiment: test_tilted (mount rpy_deg [0.0, 30.0, 0.0])"
    )
    assert isinstance(actions[1], Node)
```

`kinova_gen3_description/test/test_description_launch.py`:

```python
"""description.launch.py, live: the model on /arm/robot_description, latched, and TF
rooted at arm_mount with the mount's rotation."""

import math
import os
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

TILTED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mounts", "tilted.yaml")
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.mark.launch_test
def generate_test_description():
    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kinova_gen3_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={"mount_file": TILTED}.items(),
    )
    return LaunchDescription([desc, launch_testing.actions.ReadyToTest()])


class TestDescription(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("description_launch_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, done, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline and not done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return done()

    def test_model_is_latched_on_arm_robot_description(self):
        # Subscribe AFTER robot_state_publisher has published: only a transient-local
        # subscription still gets it, which is what the driver and cuRobo rely on.
        time.sleep(3.0)
        got = []
        self.node.create_subscription(
            String, "/arm/robot_description", lambda m: got.append(m.data), LATCHED
        )
        self.assertTrue(self._spin_until(lambda: got, 10.0), "nothing latched")
        self.assertIn('name="arm_tool_frame"', got[0])
        self.assertNotIn("ros2_control", got[0])

    def test_not_published_on_the_global_topic(self):
        # Wait until the remapped topic exists, so "absent" is not just "too early".
        def names():
            return dict(self.node.get_topic_names_and_types())

        self.assertTrue(
            self._spin_until(lambda: "/arm/robot_description" in names(), 10.0)
        )
        self.assertNotIn("/robot_description", names())

    def test_tf_carries_the_mount_rotation(self):
        buf = Buffer()
        TransformListener(buf, self.node)
        tf = []

        def lookup():
            try:
                tf.append(
                    buf.lookup_transform("arm_mount", "arm_base_link", rclpy.time.Time())
                )
            except Exception:
                pass
            return tf

        self.assertTrue(self._spin_until(lookup, 10.0), "no arm_mount -> arm_base_link")
        q = tf[0].transform.rotation
        # +30 deg about Y: (x, y, z, w) = (0, sin 15deg, 0, cos 15deg)
        self.assertAlmostEqual(q.x, 0.0, places=6)
        self.assertAlmostEqual(q.y, math.sin(math.radians(15.0)), places=6)
        self.assertAlmostEqual(q.z, 0.0, places=6)
        self.assertAlmostEqual(q.w, math.cos(math.radians(15.0)), places=6)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
```

In `kinova_gen3_description/CMakeLists.txt`, replace

```cmake
  find_package(ament_cmake_pytest REQUIRED)
```

with

```cmake
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
```

and after the `ament_add_pytest_test(model_parity …)` call add:

```cmake
  ament_add_pytest_test(description_log test/test_description_log.py ${_test_env})
  add_launch_test(test/test_description_launch.py TARGET description_launch ${_test_env})
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_description "description_log|description_launch"'
```

Expected: FAIL. `description_log` fails with `AttributeError: module 'description_launch' has no attribute '_rsp'`. `description_launch` fails at launch with `Included launch description missing required argument` because the old file has no `mount_file`; the old file also serves no `/arm/robot_description`.

- [ ] **Step 3: Rewrite the launch file**

Replace `kinova_gen3_description/launch/description.launch.py` entirely:

```python
"""Render the arm's model for one robot and run robot_state_publisher on it.

Deliberately contains nothing arm-specific and starts no driver, so a client that wants
only TF -- a planner, a perception node, RViz -- can include it without owning the arm.

mount_file is REQUIRED and has no default: it is the robot's mount.yaml (the rotation
of arm_base_link in arm_mount), and a default would silently render a level arm on a
tilted robot. The model is rendered HERE, at launch, and published on
/arm/robot_description (latched); the driver and cuRobo read it from there, so all
three use one model by construction.

robot_state_publisher DOES derive <mimic> joints -- verified 2026-09-03 with a two-joint
URDF: publishing only the driver joint moved the mimic link by exactly -0.6 rad for a
+0.6 rad drive. The model has 13 movable joints but only EIGHT independent ones (seven
arm plus arm_robotiq_85_left_knuckle_joint), all of which the driver publishes.
"""

from kinova_gen3_render import embodiment_line, render
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rsp(context):
    xml, mount = render(LaunchConfiguration("mount_file").perform(context))
    return [
        LogInfo(msg=embodiment_line(mount)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            # value_type=str: without it launch_ros guesses the type of the string
            # and robot_state_publisher can receive a mangled description.
            parameters=[{"robot_description": ParameterValue(xml, value_type=str)}],
            # The model belongs to the arm; /tf and /joint_states stay global.
            remappings=[("robot_description", "/arm/robot_description")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mount_file",
                description="the robot's mount.yaml (in containers "
                "/config/robot/mount.yaml). Required: no default.",
            ),
            OpaqueFunction(function=_rsp),
        ]
    )
```

Update the manual live script `kinova_gen3_description/test/test_tf_updates.py` so it speaks the new names. Lines 37–38 become:

```python
BASE, TOOL = "arm_mount", "arm_end_effector_link"
JOINTS = [f"arm_joint_{i}" for i in range(1, 8)]
```

At line 94, `print(f"  joint_2 = 0     -> …` becomes `print(f"  arm_joint_2 = 0     -> …`, and at line 95 `print(f"  joint_2 = pi/4  -> …` becomes `print(f"  arm_joint_2 = pi/4  -> …`. Lines 82–85 (the first `print(` call in `main`) become:

```python
            print(
                f"FAIL: no transform {BASE} -> {TOOL}. Is arm_state_publisher "
                "(description.launch.py mount_file:=...) running?"
            )
```

Line 24's usage text becomes `Run against a live `ros2 launch kinova_gen3_description description.launch.py mount_file:=\<mount.yaml>`:`.

- [ ] **Step 4: Run the tests to verify they pass**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_description "model_parity|description_log|description_launch"'
```

Expected: `Summary: 26 tests, 0 errors, 0 failures, 0 skipped`. That is 18 parity cases, 1 log case, and 3 live cases plus the exit-code check, with the result XML counted. This exact package, Tasks 1–2 applied, gave that summary on abra when this plan was written.

Then check that the required argument is enforced:

```bash
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && \
  ros2 launch kinova_gen3_description description.launch.py 2>&1 | tail -1'"
```

Expected: `[ERROR] [launch]: Caught exception in launch (see debug for traceback): Included launch description missing required argument 'mount_file' (description: 'the robot's mount.yaml (in containers /config/robot/mount.yaml). Required: no default.'), given: []`

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_description/launch/description.launch.py kinova_gen3_description/CMakeLists.txt \
  kinova_gen3_description/test/test_description_log.py \
  kinova_gen3_description/test/test_description_launch.py \
  kinova_gen3_description/test/test_tf_updates.py
git commit -m "feat(description): render at launch from mount_file, publish /arm/robot_description

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 3: Read the model from `/arm/robot_description`, and derive the `/joint_states` names from it

**Files:**

- Create: `kinova_gen3_ros2/include/kinova_gen3_ros2/robot_description.h`
- Create: `kinova_gen3_ros2/src/robot_description.cpp`
- Modify: `kinova_gen3_ros2/CMakeLists.txt:13-15` (find_package), after `:49` (library) and `:157` (test)
- Modify: `kinova_gen3_ros2/package.xml` (add `urdf`)
- Test: `kinova_gen3_ros2/test/robot_description_test.cpp`

**Interfaces:**

- Consumes: nothing from earlier tasks. The topic contract matches Task 2.

- Produces (namespace `kinova_gen3_ros2`, header `kinova_gen3_ros2/robot_description.h`, CMake target `robot_description`):

  - `inline constexpr char kRobotDescriptionTopic[] = "/arm/robot_description";`
  - `inline constexpr char kGripperJoint[] = "arm_robotiq_85_left_knuckle_joint";`
  - `std::optional<std::string> wait_for_robot_description(const rclcpp::Node::SharedPtr &node, std::chrono::milliseconds timeout);`
  - `std::vector<std::string> joint_state_names(const std::string &urdf_xml, std::vector<std::string> arm_joints);`, which throws `std::runtime_error` that names the missing joint.

- [ ] **Step 1: Write the failing test**

`kinova_gen3_ros2/test/robot_description_test.cpp`:

```cpp
// The driver's model arrives on /arm/robot_description, not as a file.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "kinova_gen3_ros2/robot_description.h"

using namespace std::chrono_literals;
using kinova_gen3_ros2::joint_state_names;
using kinova_gen3_ros2::wait_for_robot_description;

namespace {
// Smallest URDF urdf::Model accepts, with one named revolute joint.
std::string urdf_with(const std::string &joint) {
  return "<robot name='t'><link name='a'/><link name='b'/>"
         "<joint name='" +
         joint +
         "' type='revolute'><parent link='a'/><child link='b'/>"
         "<axis xyz='0 0 1'/><limit lower='0' upper='0.8' effort='1' "
         "velocity='1'/></joint></robot>";
}
const std::vector<std::string> kArm = {"arm_joint_1", "arm_joint_2",
                                       "arm_joint_3", "arm_joint_4",
                                       "arm_joint_5", "arm_joint_6",
                                       "arm_joint_7"};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
latched_publisher(const rclcpp::Node::SharedPtr &n) {
  return n->create_publisher<std_msgs::msg::String>(
      "/arm/robot_description", rclcpp::QoS(1).reliable().transient_local());
}
} // namespace

class RobotDescriptionTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

// Published BEFORE the driver subscribes -- arm_state_publisher started first.
// Only a transient-local subscription still gets it.
TEST_F(RobotDescriptionTest, ALatchedDescriptionPublishedEarlierArrives) {
  auto pub_node = std::make_shared<rclcpp::Node>("fake_asp_early");
  auto pub = latched_publisher(pub_node);
  std_msgs::msg::String m;
  m.data = urdf_with("arm_robotiq_85_left_knuckle_joint");
  pub->publish(m);

  auto node = std::make_shared<rclcpp::Node>("driver_early");
  const auto got = wait_for_robot_description(node, 5s);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, m.data);
}

// Published AFTER the driver starts waiting -- the driver started first.
TEST_F(RobotDescriptionTest, PublishedAfterTheDriverStartsWaitingArrives) {
  auto pub_node = std::make_shared<rclcpp::Node>("fake_asp_late");
  auto pub = latched_publisher(pub_node);
  std::thread late([&pub] {
    std::this_thread::sleep_for(500ms);
    std_msgs::msg::String m;
    m.data = "late";
    pub->publish(m);
  });
  auto node = std::make_shared<rclcpp::Node>("driver_late");
  const auto got = wait_for_robot_description(node, 5s);
  late.join();
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, "late");
}

TEST_F(RobotDescriptionTest, NoPublisherTimesOut) {
  auto node = std::make_shared<rclcpp::Node>("driver_alone");
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(wait_for_robot_description(node, 300ms).has_value());
  EXPECT_GE(std::chrono::steady_clock::now() - t0, 300ms);
}

TEST(JointStateNames, ArmJointsThenTheGripperJoint) {
  const auto names =
      joint_state_names(urdf_with("arm_robotiq_85_left_knuckle_joint"), kArm);
  ASSERT_EQ(names.size(), 8u);
  EXPECT_EQ(names.front(), "arm_joint_1");
  EXPECT_EQ(names[6], "arm_joint_7");
  EXPECT_EQ(names.back(), "arm_robotiq_85_left_knuckle_joint");
}

// An unprefixed (old) model on the topic: refuse, and say which joint.
TEST(JointStateNames, AModelWithoutTheGripperJointIsRefusedByName) {
  try {
    joint_state_names(urdf_with("robotiq_85_left_knuckle_joint"), kArm);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("arm_robotiq_85_left_knuckle_joint"),
              std::string::npos)
        << e.what();
  }
}

TEST(JointStateNames, GarbageIsRefused) {
  EXPECT_THROW(joint_state_names("not xml", kArm), std::runtime_error);
}
```

In `kinova_gen3_ros2/CMakeLists.txt`, before `endif()` at line 158, add:

```cmake
  ament_add_gtest(robot_description_test test/robot_description_test.cpp)
  target_link_libraries(robot_description_test robot_description)
  ament_target_dependencies(robot_description_test rclcpp std_msgs)
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-driver-mount/ abra:mount_ws/src/kinova-gen3-driver/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 robot_description_test'
```

Expected: FAIL. The CMake configure fails with `Target "robot_description_test" links to target "robot_description" but the target was not found`, or the compile fails with `kinova_gen3_ros2/robot_description.h: No such file or directory`.

- [ ] **Step 3: Write the header and source**

`kinova_gen3_ros2/include/kinova_gen3_ros2/robot_description.h`:

```cpp
// kinova_gen3_ros2/include/kinova_gen3_ros2/robot_description.h
#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
namespace kinova_gen3_ros2 {

// arm_state_publisher (kinova_gen3_description's description.launch.py) renders
// the model once, at launch, from the robot's mount file and publishes it here,
// latched. The driver reads it rather than a URDF file, so it cannot disagree
// with TF or with cuRobo -- including about the mount rotation, which is what
// gravity compensation depends on.
inline constexpr char kRobotDescriptionTopic[] = "/arm/robot_description";
// The 2F-85's one actuated joint; robot_state_publisher derives its five mimics.
inline constexpr char kGripperJoint[] = "arm_robotiq_85_left_knuckle_joint";

// Block until one message arrives on kRobotDescriptionTopic (RELIABLE +
// TRANSIENT_LOCAL, so a description published before we subscribed still
// arrives), or until `timeout`. nullopt on timeout or shutdown. Spins `node` on
// a private executor: call it before the node joins any other executor.
std::optional<std::string>
wait_for_robot_description(const rclcpp::Node::SharedPtr &node,
                           std::chrono::milliseconds timeout);

// The names /joint_states carries: `arm_joints` (the model's arm joints in
// chain order -- Dynamics::joint_names()) followed by kGripperJoint. Throws
// std::runtime_error naming the joint if the URDF does not declare it:
// robot_state_publisher matches by NAME, and a name it does not know freezes
// that part of TF silently.
std::vector<std::string>
joint_state_names(const std::string &urdf_xml,
                  std::vector<std::string> arm_joints);
} // namespace kinova_gen3_ros2
```

`kinova_gen3_ros2/src/robot_description.cpp`:

```cpp
// kinova_gen3_ros2/src/robot_description.cpp
#include "kinova_gen3_ros2/robot_description.h"
#include <stdexcept>
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"
namespace kinova_gen3_ros2 {

std::optional<std::string>
wait_for_robot_description(const rclcpp::Node::SharedPtr &node,
                           std::chrono::milliseconds timeout) {
  std::optional<std::string> xml;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      kRobotDescriptionTopic, rclcpp::QoS(1).reliable().transient_local(),
      [&xml](std_msgs::msg::String::ConstSharedPtr m) { xml = m->data; });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!xml && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    ex.spin_some(std::chrono::milliseconds(50));
  ex.remove_node(node);
  return xml;
}

std::vector<std::string>
joint_state_names(const std::string &urdf_xml,
                  std::vector<std::string> arm_joints) {
  urdf::Model model;
  if (!model.initString(urdf_xml))
    throw std::runtime_error(std::string(kRobotDescriptionTopic) +
                             " is not a parseable URDF");
  if (!model.getJoint(kGripperJoint))
    throw std::runtime_error(std::string(kRobotDescriptionTopic) +
                             " has no joint '" + kGripperJoint +
                             "': is arm_state_publisher serving the arm_ model?");
  arm_joints.emplace_back(kGripperJoint);
  return arm_joints;
}
} // namespace kinova_gen3_ros2
```

In `kinova_gen3_ros2/CMakeLists.txt`, after line 15 (`find_package(diagnostic_updater REQUIRED)`) add:

```cmake
find_package(urdf REQUIRED)
```

and after line 49, the end of the `ros2_backend` block, add:

```cmake
# Reads the latched model arm_state_publisher renders, and derives the
# /joint_states names from it. Used once, at startup, before the RT loop.
add_library(robot_description src/robot_description.cpp)
target_include_directories(robot_description PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(robot_description rclcpp std_msgs urdf)
```

In `kinova_gen3_ros2/package.xml`, after `<depend>std_msgs</depend>` add `<depend>urdf</depend>`.

- [ ] **Step 4: Run the test to verify it passes**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 robot_description_test'
```

Expected: `0 errors, 0 failures`, with the gtest XML listing 6 tests passed. Before this plan was written, this exact header, source and test file passed (6/6) in a scratch package on abra.

The build of `kinova_gen3_ros2` itself still uses the old `Dynamics(path)` in `bringup_node.cpp` at this point. That is fine, because the file constructor still exists.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/include/kinova_gen3_ros2/robot_description.h \
  kinova_gen3_ros2/src/robot_description.cpp kinova_gen3_ros2/test/robot_description_test.cpp \
  kinova_gen3_ros2/CMakeLists.txt kinova_gen3_ros2/package.xml
git commit -m "feat(node): wait for /arm/robot_description; joint_states names from the model

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 4: The node builds Dynamics and `/joint_states` from the model; `--urdf` removed

**Files:**

- Modify: `kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h:3-5, 24, 76-77`
- Modify: `kinova_gen3_ros2/src/ros2_backend.cpp:3-4, 13, 189-190, 218`
- Modify: `kinova_gen3_ros2/src/bringup_node.cpp:1-111, 171-172, 195-199`
- Modify: `kinova_gen3_ros2/CMakeLists.txt:80-82` (link `robot_description` into the node)
- Modify: `kinova_gen3_description/launch/bringup.launch.py` (whole file)
- Test: `kinova_gen3_ros2/test/ros2_backend_test.cpp`, `kinova_gen3_ros2/test/gripper_server_test.cpp`

**Interfaces:**

- Consumes: `kinova_gen3_ros2::wait_for_robot_description`, `joint_state_names`, `kRobotDescriptionTopic` (Task 3). From the driver: `kinova::UrdfXml { std::string xml; }`, `Dynamics(const UrdfXml&, const std::string& ee_frame)`, `std::vector<std::string> Dynamics::joint_names() const`.

- Produces:

  - `Ros2Backend(rclcpp::Node::SharedPtr node, std::vector<std::string> joint_names)`. It requires `kNumJoints + 1` names (7 arm joints, then the gripper joint) and otherwise throws `std::invalid_argument`.
  - Node CLI: `--urdf` is gone. `--ee-frame` defaults to `arm_end_effector_link`.
  - `bringup.launch.py` requires `mount_file`.

- [ ] **Step 1: Write the failing tests**

`kinova_gen3_ros2/test/ros2_backend_test.cpp`: add `#include <stdexcept>`, `#include <string>` and `#include <vector>` after line 5 (`#include <thread>`). Inside the anonymous namespace, after `one_point_goal` (line 24), add:

```cpp
const std::vector<std::string> kJointNames = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7",
    "arm_robotiq_85_left_knuckle_joint"};
```

Replace each of the four `kinova_gen3_ros2::Ros2Backend backend(node);` (lines 58, 69, 81, 106) with `kinova_gen3_ros2::Ros2Backend backend(node, kJointNames);`, and append:

```cpp
// The names come from the model; a list that is not seven arm joints plus the
// gripper joint is a wiring bug, refused at construction rather than published.
TEST_F(Ros2BackendTest, AWrongNumberOfJointNamesIsRefused) {
  auto node = std::make_shared<rclcpp::Node>("backend_names_bad");
  EXPECT_THROW((void)kinova_gen3_ros2::Ros2Backend(
                   node, std::vector<std::string>{"arm_joint_1"}),
               std::invalid_argument);
}
```

`kinova_gen3_ros2/test/gripper_server_test.cpp`: add `#include <string>` and `#include <vector>` after line 3 (`#include <stdexcept>`). After `using namespace std::chrono_literals;` (line 15), add:

```cpp
static const std::vector<std::string> kJointNames = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7",
    "arm_robotiq_85_left_knuckle_joint"};
```

Replace both `std::make_shared<kinova_gen3_ros2::Ros2Backend>(node_)` (lines 151 and 180) with `std::make_shared<kinova_gen3_ros2::Ros2Backend>(node_, kJointNames)`. In `KnuckleJointIsPublishedEvenWhenAbsent`, replace line 172:

```cpp
  EXPECT_EQ(got.name[7], "robotiq_85_left_knuckle_joint");
```

with

```cpp
  EXPECT_EQ(got.name[0], "arm_joint_1"); // from the model, not hard-coded
  EXPECT_EQ(got.name[6], "arm_joint_7");
  EXPECT_EQ(got.name[7], "arm_robotiq_85_left_knuckle_joint");
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-driver-mount/ abra:mount_ws/src/kinova-gen3-driver/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 "ros2_backend_test|gripper_server_test"'
```

Expected: FAIL to compile with `no matching function for call to 'kinova_gen3_ros2::Ros2Backend::Ros2Backend(std::shared_ptr<rclcpp::Node>&, const std::vector<std::__cxx11::basic_string<char> >&)'`.

- [ ] **Step 3: Inject the names into `Ros2Backend`**

`kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h`: after line 5 (`#include <mutex>`) add `#include <string>` and `#include <vector>`. Replace line 24:

```cpp
  explicit Ros2Backend(rclcpp::Node::SharedPtr node);
```

with

```cpp
  // joint_names: the model's seven arm joints in chain order, then the gripper
  // joint -- joint_state_names() in robot_description.h. They are what
  // /joint_states carries, and robot_state_publisher matches them BY NAME, so
  // they come from the model rather than from this file. Throws
  // std::invalid_argument unless there are exactly kNumJoints + 1.
  Ros2Backend(rclcpp::Node::SharedPtr node,
              std::vector<std::string> joint_names);
```

After line 77 (`kinova::interface::GripperSink *gripper_ = nullptr;`) add:

```cpp
  std::vector<std::string> arm_joint_names_; // kNumJoints, chain order
  std::string gripper_joint_name_;
```

`kinova_gen3_ros2/src/ros2_backend.cpp`: after line 4 (`#include <limits>`) add `#include <stdexcept>`. Replace line 13:

```cpp
Ros2Backend::Ros2Backend(rclcpp::Node::SharedPtr node) : node_(node) {
```

with

```cpp
Ros2Backend::Ros2Backend(rclcpp::Node::SharedPtr node,
                         std::vector<std::string> joint_names)
    : node_(node) {
  if (joint_names.size() != static_cast<size_t>(kinova::kNumJoints) + 1)
    throw std::invalid_argument(
        "Ros2Backend: need " + std::to_string(kinova::kNumJoints) +
        " arm joint names then the gripper joint, got " +
        std::to_string(joint_names.size()));
  gripper_joint_name_ = joint_names.back();
  joint_names.pop_back();
  arm_joint_names_ = std::move(joint_names);
```

Replace lines 189–190:

```cpp
  msg.name = {"joint_1", "joint_2", "joint_3", "joint_4",
              "joint_5", "joint_6", "joint_7"};
```

with

```cpp
  msg.name = arm_joint_names_;
```

Replace line 218:

```cpp
    msg.name.push_back("robotiq_85_left_knuckle_joint");
```

with

```cpp
    msg.name.push_back(gripper_joint_name_);
```

- [ ] **Step 4: Wire the node to the topic**

`kinova_gen3_ros2/CMakeLists.txt` lines 80–82: add `robot_description` to the node's `target_link_libraries`:

```cmake
target_link_libraries(kinova_gen3_node
  ros2_backend robot_description planned_move_servers goal_router curobo_plan_client
  message_mapping arbitration_server stream_server gripper_server kinova_lowlevel::kinova_lowlevel)
```

`kinova_gen3_ros2/src/bringup_node.cpp`:

After line 2 (`#include <atomic>`) add `#include <chrono>`. After line 11 (`#include "kinova_gen3_ros2/ros2_backend.h"`) add `#include "kinova_gen3_ros2/robot_description.h"`.

Inside the anonymous namespace, after line 41 (`void on_sigint…`), add:

```cpp
// How long to wait for arm_state_publisher's model. sheppy starts containers
// concurrently, so it may come up a little after us; the topic is latched, so
// order does not matter within this window.
constexpr std::chrono::seconds kDescriptionTimeout{30};
```

Replace lines 72–78:

```cpp
  std::string urdf = "models/gen3_7dof_2f85.urdf", ip;
  // Dynamics resolves the EE frame BY NAME and throws if it is absent, so the
  // default has to match whichever model is loaded. Core's own default is
  // gen3_end_effector_link, matching the hand-edited models/ URDF its tests
  // use; kinova_gen3_description generates end_effector_link. Overridable
  // rather than hard-coded so both models work from this one binary.
  std::string ee_frame = "gen3_end_effector_link";
```

with

```cpp
  std::string ip;
  // Dynamics resolves the EE frame BY NAME and throws if it is absent. The
  // model comes from /arm/robot_description, where every link carries the arm_
  // prefix. Always passed explicitly: core's own default names its test model.
  std::string ee_frame = "arm_end_effector_link";
```

Delete lines 95–96:

```cpp
    else if (a == "--urdf")
      urdf = nxt();
```

Replace line 111:

```cpp
  Dynamics dyn(urdf, ee_frame), pump_dyn(urdf, ee_frame);
```

with

```cpp
  // The model, from arm_state_publisher, BEFORE anything real-time exists: the
  // RT loop must never wait on DDS. It was rendered from the robot's mount
  // file, so gravity compensation, FK and the Jacobian are all in arm_mount.
  auto node = std::make_shared<rclcpp::Node>("kinova_gen3_node");
  const auto urdf_xml =
      kinova_gen3_ros2::wait_for_robot_description(node, kDescriptionTimeout);
  if (!urdf_xml) {
    if (rclcpp::ok()) // a SIGTERM during the wait is a clean stop, not this
      RCLCPP_FATAL(
          node->get_logger(),
          "no model on %s after %lld s. It is published by "
          "arm_state_publisher: ros2 launch kinova_gen3_description "
          "description.launch.py mount_file:=<robot>/mount.yaml",
          kinova_gen3_ros2::kRobotDescriptionTopic,
          static_cast<long long>(kDescriptionTimeout.count()));
    rclcpp::shutdown();
    return 1;
  }
  // Core reduces the 13-DOF model to the seven joints on the chain to
  // ee_frame, locking the gripper at q = 0 -- which lumps its mass into the
  // wrist exactly as the old frozen model did.
  Dynamics dyn(UrdfXml{*urdf_xml}, ee_frame),
      pump_dyn(UrdfXml{*urdf_xml}, ee_frame);
  std::vector<std::string> js_names;
  try {
    js_names = kinova_gen3_ros2::joint_state_names(*urdf_xml, dyn.joint_names());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
```

Replace lines 171–172:

```cpp
  auto node = std::make_shared<rclcpp::Node>("kinova_gen3_node");
  auto backend = std::make_shared<kinova_gen3_ros2::Ros2Backend>(node);
```

with

```cpp
  auto backend =
      std::make_shared<kinova_gen3_ros2::Ros2Backend>(node, js_names);
```

Lines 195–198 describe a model that no longer exists. Replace

```cpp
  // "Expected" cannot be inferred from the node's own model: it loads the
  // FROZEN 7-DOF URDF, in which the Robotiq joints are type="fixed", so its
  // model never has a gripper regardless of the hardware. A robot genuinely
  // built without one sets this false.
```

with

```cpp
  // "Expected" cannot be inferred from the node's own model: core reduces it to
  // the seven arm joints, so it never has a gripper DOF regardless of the
  // hardware. A robot genuinely built without one sets this false.
```

`kinova_gen3_description/launch/bringup.launch.py`: replace it entirely:

```python
"""Start the driver and TF together, on one model.

description.launch.py renders the model from mount_file and publishes it on
/arm/robot_description; the driver reads it from there. So robot_state_publisher and
the driver cannot disagree -- which is the point of the package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("kinova_gen3_description")
    sim = LaunchConfiguration("sim")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "description.launch.py"])
        ),
        launch_arguments={"mount_file": LaunchConfiguration("mount_file")}.items(),
    )

    params = [
        {
            "arbitration_mode": LaunchConfiguration("arbitration_mode"),
            "estop_clear_max_age_s": LaunchConfiguration("estop_clear_max_age_s"),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mount_file",
                description="the robot's mount.yaml. Required: no default.",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="run against SimTransport instead of the arm",
            ),
            DeclareLaunchArgument(
                "ip",
                default_value="192.168.1.10",
                description="arm IP; used only when sim:=false",
            ),
            # Dynamics resolves this by name and throws if it is absent from the model.
            DeclareLaunchArgument("ee_frame", default_value="arm_end_effector_link"),
            DeclareLaunchArgument(
                "arbitration_mode",
                default_value="disabled",
                description="enforced | disabled; read-only at runtime",
            ),
            DeclareLaunchArgument("estop_clear_max_age_s", default_value="1.0"),
            description,
            # Two Nodes rather than one with a conditional argument list: the driver takes
            # --sim OR --ip, never both, and launch substitutions cannot build a variable
            # length argv.
            Node(
                package="kinova_gen3_ros2",
                executable="kinova_gen3_node",
                output="screen",
                condition=IfCondition(sim),
                arguments=["--sim", "--ee-frame", LaunchConfiguration("ee_frame")],
                parameters=params,
            ),
            Node(
                package="kinova_gen3_ros2",
                executable="kinova_gen3_node",
                output="screen",
                condition=UnlessCondition(sim),
                arguments=[
                    "--ip",
                    LaunchConfiguration("ip"),
                    "--ee-frame",
                    LaunchConfiguration("ee_frame"),
                ],
                parameters=params,
            ),
        ]
    )
```

- [ ] **Step 5: Run the tests to verify they pass**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-driver-mount/ abra:mount_ws/src/kinova-gen3-driver/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 "ros2_backend_test|gripper_server_test|robot_description_test"'
```

Expected: `0 errors, 0 failures`.

Then run the node end-to-end in sim (the arm is off; `--sim` only), against the description from Task 2:

```bash
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && export ROS_DOMAIN_ID=79 && \
  (ros2 launch kinova_gen3_description bringup.launch.py sim:=true \
     mount_file:=\$HOME/mount_ws/src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/tilted.yaml > /tmp/bringup.log 2>&1 &) && \
  sleep 12 && ros2 topic echo --once --qos-reliability best_effort /joint_states --field name && \
  grep -m1 -o \"embodiment: .*\" /tmp/bringup.log; grep -m1 -o \"kinova_gen3_node up (sim)\" /tmp/bringup.log; \
  pkill -INT -f \"ros2 launch kinova_gen3_description\"'"
```

Expected output:

```
- arm_joint_1
- arm_joint_2
- arm_joint_3
- arm_joint_4
- arm_joint_5
- arm_joint_6
- arm_joint_7
- arm_robotiq_85_left_knuckle_joint
---
embodiment: test_tilted (mount rpy_deg [0.0, 30.0, 0.0])
kinova_gen3_node up (sim)
```

Then check the timeout path (no description running):

```bash
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && export ROS_DOMAIN_ID=79 && \
  ~/mount_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node --sim 2>&1 | tail -1; echo rc=\$?'"
```

Expected after about 30 s: `[FATAL] [...] [kinova_gen3_node]: no model on /arm/robot_description after 30 s. It is published by arm_state_publisher: ros2 launch kinova_gen3_description description.launch.py mount_file:=<robot>/mount.yaml`. (`rc` reflects `tail`. Exit code 1 is checked by the smoke test in Task 6.)

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h kinova_gen3_ros2/src/ros2_backend.cpp \
  kinova_gen3_ros2/src/bringup_node.cpp kinova_gen3_ros2/CMakeLists.txt \
  kinova_gen3_ros2/test/ros2_backend_test.cpp kinova_gen3_ros2/test/gripper_server_test.cpp \
  kinova_gen3_description/launch/bringup.launch.py
git commit -m "feat(node)!: model from /arm/robot_description; --urdf removed

Dynamics is built from the latched XML (core reduces it to the arm chain);
/joint_states names come from the model; --ee-frame defaults to
arm_end_effector_link.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 5: `GoToEEPose` goals are in `arm_mount`

**Files:**

- Modify: `kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h:59` (add a declaration after it)
- Modify: `kinova_gen3_ros2/src/message_mapping.cpp:183` (add a definition after `speed_scale_rejection`)
- Modify: `kinova_gen3_ros2/include/kinova_gen3_ros2/goto_ee_pose_server.h:25-27`
- Test: `kinova_gen3_ros2/test/message_mapping_test.cpp` (append), `kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp` (8 frame strings and a new test)

**Interfaces:**

- Consumes: nothing from earlier tasks.

- Produces: `std::optional<std::string> kinova_gen3_ros2::ee_pose_frame_rejection(const std::string &frame_id);`, which returns nullopt for exactly `"arm_mount"`.

- [ ] **Step 1: Write the failing tests**

Append to `kinova_gen3_ros2/test/message_mapping_test.cpp`:

```cpp
// GoToEEPose targets are in arm_mount (gravity-aligned on level ground). base_link
// gets its own message because it was the old contract and every un-migrated
// client sends it; the client only sees "rejected", so this text is for the log.
TEST(MessageMapping, EePoseFrameMustBeArmMount) {
  EXPECT_EQ(ee_pose_frame_rejection("arm_mount"), std::nullopt);
  EXPECT_EQ(ee_pose_frame_rejection("base_link"),
            std::optional<std::string>(
                "frame_id 'base_link' is no longer accepted: Cartesian goals "
                "are in 'arm_mount' (gravity-aligned); arm_base_link is tilted "
                "30° on flatbot and gen_1_5"));
  EXPECT_EQ(ee_pose_frame_rejection("arm_base_link"),
            std::optional<std::string>("frame_id 'arm_base_link' != arm_mount"));
  // A default-constructed goal: the client forgot the header.
  EXPECT_EQ(ee_pose_frame_rejection(""),
            std::optional<std::string>("frame_id '' != arm_mount"));
}
```

In `kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp`, change every goal frame from `base_link` to `arm_mount`:

```bash
sed -i.bak 's/"base_link"/"arm_mount"/g' kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp && \
  rm kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp.bak
grep -c '"arm_mount"' kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp
```

Expected: `8`.

Then, after `PlanSuccessDrivesTrajectoryAndSucceeds` (it ends at line 203), add:

```cpp
// The old frame is refused at the goal gate, before any planning: an
// un-migrated client must not have its base_link numbers silently read in
// arm_mount (they differ by 30° on flatbot).
TEST_F(GotoServerTest, ABaseLinkGoalIsRejectedBeforePlanning) {
  auto node = std::make_shared<rclcpp::Node>("goto_it_base_link");
  kinova_gen3_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true,
                                                /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_gen3_ros2::GoalRouter router(dummy);
  kinova_gen3_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  SpinThread spin(ex);

  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = "base_link";
  auto gh_fut = client->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(gh_fut.get(), nullptr); // rejected
  EXPECT_FALSE(sup.got_goal);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 "message_mapping_test|goto_ee_pose_integration_test"'
```

Expected: FAIL. `message_mapping_test` does not compile (`'ee_pose_frame_rejection' was not declared in this scope`), so the run reports a build failure.

- [ ] **Step 3: Implement**

`kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h`: after line 59 (`std::optional<std::string> speed_scale_rejection(double s);`) add:

```cpp

// Why a GoToEEPose target in `frame_id` is unacceptable, or nullopt. Goals are
// in arm_mount: gravity-aligned on level ground, origin at the arm's base.
// base_link -- the old contract -- gets a message of its own saying so.
std::optional<std::string> ee_pose_frame_rejection(const std::string &frame_id);
```

`kinova_gen3_ros2/src/message_mapping.cpp`: after line 183 (the closing `}` of `speed_scale_rejection`) add:

```cpp

std::optional<std::string> ee_pose_frame_rejection(const std::string &frame_id) {
  if (frame_id == "arm_mount")
    return std::nullopt;
  if (frame_id == "base_link")
    return std::string(
        "frame_id 'base_link' is no longer accepted: Cartesian goals are in "
        "'arm_mount' (gravity-aligned); arm_base_link is tilted 30° on flatbot "
        "and gen_1_5");
  return "frame_id '" + frame_id + "' != arm_mount";
}
```

`kinova_gen3_ros2/include/kinova_gen3_ros2/goto_ee_pose_server.h`: replace lines 25–27:

```cpp
    if (goal.target.header.frame_id != "base_link")
      return "GoToEEPose: frame_id '" + goal.target.header.frame_id +
             "' != base_link";
```

with

```cpp
    if (auto why = ee_pose_frame_rejection(goal.target.header.frame_id))
      return "GoToEEPose: " + *why;
```

(`planned_move_server.h` already includes `message_mapping.h`, which is where `speed_scale_rejection` reaches this file from.)

- [ ] **Step 4: Run the tests to verify they pass**

Run:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 "message_mapping_test|goto_ee_pose_integration_test"'
```

Expected: `0 errors, 0 failures`.

Then run the whole package, which is the CONTRIBUTING gate. On `dev`, with the interfaces from the setup above, `kinova_gen3_ros2` plus `kinova_gen3_description` report 132 test cases on abra. After Tasks 3–5 there are 9 more gtest cases in this package:

```bash
ssh abra 'bash ~/mount_ws/run.sh kinova_gen3_ros2 "."'
```

Expected: `0 errors, 0 failures`.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h kinova_gen3_ros2/src/message_mapping.cpp \
  kinova_gen3_ros2/include/kinova_gen3_ros2/goto_ee_pose_server.h \
  kinova_gen3_ros2/test/message_mapping_test.cpp kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp
git commit -m "feat(goto)!: GoToEEPose goals are in arm_mount; base_link refused by name

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 6: The image ships the description; every run path starts it first

**Files:**

- Modify: `Dockerfile:85` (COPY), `:141-144, 149` (colcon), `:159` (ENV URDF), `:170-171` (CMD)
- Modify: `Makefile:8, 33, 49, 64-65, 69-72, 86-87, 119-122`
- Modify: `scripts/smoke.sh:17-26`
- Modify: `deploy/compose.yaml:34-35` (driver command); add `arm_state_publisher` service
- Create: `rammp-alternative.description.yaml`

**Interfaces:**

- Consumes: `description.launch.py mount_file:=…` (Task 2) and the node without `--urdf` (Task 4).

- Produces:

  - The image contains `kinova_gen3_description` and its run dependencies (xacro, kortex/robotiq/realsense descriptions).
  - Image command: `kinova_gen3_node --sim`.
  - Description container command: `ros2 launch kinova_gen3_description description.launch.py mount_file:=/config/robot/mount.yaml`.
  - `make real` requires `MOUNT=<file>`.

- [ ] **Step 1: Write the failing check**

The smoke test is the check. Make it start the description container first and assert both banners. Replace lines 17–26 of `scripts/smoke.sh`:

```bash
PATTERN="${SMOKE_PATTERN:-kinova_gen3_node up}"

IMAGE="${1:-kinova-gen3-ros2:humble}"
NAME="rammp-smoke-$$"

docker run -d --rm --name "$NAME" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT

sleep 5
```

with

```bash
PATTERN="${SMOKE_PATTERN:-kinova_gen3_node up}"

IMAGE="${1:-kinova-gen3-ros2:humble}"
NAME="rammp-smoke-$$"
# The node reads its model from /arm/robot_description and exits without it, so
# arm_state_publisher runs first, from the same image, on the level TEST fixture
# bind-mounted where a robot's mount.yaml goes.
MOUNT="$(cd "$(dirname "$0")/.." && pwd)/kinova_gen3_description/test/mounts/level.yaml"

docker run -d --rm --name "$NAME-desc" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "$MOUNT:/config/robot/mount.yaml:ro" "$IMAGE" \
  ros2 launch kinova_gen3_description description.launch.py \
  mount_file:=/config/robot/mount.yaml >/dev/null
trap 'docker rm -f "$NAME" "$NAME-desc" >/dev/null 2>&1 || true' EXIT

docker run -d --rm --name "$NAME" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" >/dev/null

sleep 10

if ! docker logs "$NAME-desc" 2>&1 | grep -q "embodiment: test_level"; then
  echo "FAIL: arm_state_publisher did not log its embodiment line"
  docker logs "$NAME-desc" 2>&1 | tail -20
  exit 1
fi
```

- [ ] **Step 2: Run it to verify it fails**

The image is built on abra from the rsynced tree. `CORE_REF` must be a ref GitHub can serve: the driver's `feat/tilted-arm-mount` branch, once it is on GitHub. If `git ls-remote --heads https://github.com/rammp-org/kinova-gen3-driver.git feat/tilted-arm-mount` prints nothing, stop and ask whoever owns the driver branch to publish it. Do not push it yourself.

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'cd ~/mount_ws/src/kinova_gen3_ros2 && make build CORE_REF=feat/tilted-arm-mount && ROS_DOMAIN_ID=79 make smoke'
```

Expected: FAIL. The image has no `kinova_gen3_description` (the released 1.1.0 image's `/module_ws/install` holds only `kinova_gen3_ros2`, `kinova_lowlevel` and `rammp_curobo_interfaces`), so the output is `FAIL: arm_state_publisher did not log its embodiment line` with `Package 'kinova_gen3_description' not found` in the tail.

- [ ] **Step 3: Build the description into the image**

`Dockerfile`: after line 85 (`COPY kinova_gen3_ros2/package.xml       src/kinova_gen3_ros2/kinova_gen3_ros2/package.xml`) add:

```dockerfile
# The description too: arm_state_publisher runs from THIS image and renders the
# model at launch, so xacro and the upstream descriptions are run-time deps.
COPY kinova_gen3_description/package.xml src/kinova_gen3_ros2/kinova_gen3_description/package.xml
```

(`rosdep --from-paths src/kinova_gen3_ros2` already recurses into it.)

Replace lines 141–144:

```dockerfile
# --packages-up-to kinova_gen3_ros2 builds exactly the node and its recursive
# deps (kinova_lowlevel, rammp_arm_interfaces, rammp_common_interfaces,
# rammp_curobo_interfaces) and stops there — rammp_curobo_ros is the GPU planner node and belongs in the
# rammp-curobo image, not this one.
```

with

```dockerfile
# --packages-up-to kinova_gen3_ros2 kinova_gen3_description builds the node, the
# model arm_state_publisher renders at launch, and their recursive deps
# (kinova_lowlevel, rammp_arm_interfaces, rammp_common_interfaces,
# rammp_curobo_interfaces) and stops there — rammp_curobo_ros is the GPU planner
# node and belongs in the rammp-curobo image, not this one.
```

and on line 149 replace `      --packages-up-to kinova_gen3_ros2 \` with `      --packages-up-to kinova_gen3_ros2 kinova_gen3_description \`.

Delete line 159 (`ENV URDF=/module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf`) and the blank line after it.

Replace lines 170–171:

```dockerfile
CMD ["/module_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node", \
     "--sim", "--urdf", "/module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf"]
```

with

```dockerfile
# No --urdf: the node waits for /arm/robot_description from arm_state_publisher
# (same image: ros2 launch kinova_gen3_description description.launch.py
# mount_file:=/config/robot/mount.yaml) and exits if it never comes.
CMD ["/module_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node", "--sim"]
```

`Makefile`: delete line 33 (`URDF   := /module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf`). Replace line 49 (`NODE_ARGS := --urdf $(URDF) --cpu $(RT_CORE)`) with:

```make
NODE_ARGS := --cpu $(RT_CORE)

# The node reads its model from /arm/robot_description, which arm_state_publisher
# (this image, kinova_gen3_description) renders from the robot's mount file. So
# every run target starts that container first. The sim targets use the level TEST
# fixture; `real` requires MOUNT -- no default, because a wrong mount silently
# applies the wrong gravity.
SIM_MOUNT := $(CURDIR)/kinova_gen3_description/test/mounts/level.yaml
DESC_CMD  := ros2 launch kinova_gen3_description description.launch.py mount_file:=/config/robot/mount.yaml
# $(call desc_up,<container name>,<mount file>,<image>)
desc_up = docker run -d --rm --name $(1) $(ROS_FLAGS) -v $(2):/config/robot/mount.yaml:ro $(3) $(DESC_CMD) >/dev/null
```

Replace lines 64–65, the `sim` target:

```make
sim: build                 ## Run the node in sim, foreground
	$(RUN) -it --name kinova_gen3_sim $(IMAGE) $(NODE) --sim $(NODE_ARGS)
```

with

```make
sim: build                 ## Run the node in sim, foreground
	$(call desc_up,kinova_gen3_sim_desc,$(SIM_MOUNT),$(IMAGE))
	-$(RUN) -it --name kinova_gen3_sim $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	docker rm -f kinova_gen3_sim_desc >/dev/null
```

Replace lines 69–72, the head of `e2e`:

```make
e2e: build                 ## Sim integration check (success + path-tolerance abort)
	$(RUN) -d --name kinova_gen3_e2e $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	@sleep 5
	@set -e; trap 'docker rm -f kinova_gen3_e2e >/dev/null' EXIT; \
```

with

```make
e2e: build                 ## Sim integration check (success + path-tolerance abort)
	$(call desc_up,kinova_gen3_e2e_desc,$(SIM_MOUNT),$(IMAGE))
	$(RUN) -d --name kinova_gen3_e2e $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	@sleep 8
	@set -e; trap 'docker rm -f kinova_gen3_e2e kinova_gen3_e2e_desc >/dev/null' EXIT; \
```

Replace lines 86–87, the `run` target:

```make
run:                       ## Run the sim image, the way the standard does
	$(RUN) -it $(IMAGE) $(NODE) --sim $(NODE_ARGS)
```

with

```make
run:                       ## Run the sim image, the way the standard does
	$(call desc_up,kinova_gen3_run_desc,$(SIM_MOUNT),$(IMAGE))
	-$(RUN) -it $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	docker rm -f kinova_gen3_run_desc >/dev/null
```

Replace lines 119–122, the `real` target:

```make
real: build-real           ## Build KORTEX-enabled and run against the arm
	@test -n "$(IP)" || { echo "usage: make real IP=192.168.1.10"; exit 1; }
	$(RUN) -it --name kinova_gen3_real --stop-timeout 20 $(IMAGE_REAL) \
	  $(NODE) --ip $(IP) $(NODE_ARGS)
```

with

```make
real: build-real           ## Build KORTEX-enabled and run against the arm
	@test -n "$(IP)" -a -f "$(MOUNT)" || { echo "usage: make real IP=192.168.1.10" \
	  "MOUNT=<rammp-deployments>/december_2026/config/robots/current/mount.yaml" \
	  "(MOUNT has no default: it sets the gravity direction)"; exit 1; }
	$(call desc_up,kinova_gen3_real_desc,$(abspath $(MOUNT)),$(IMAGE_REAL))
	-$(RUN) -it --name kinova_gen3_real --stop-timeout 20 $(IMAGE_REAL) \
	  $(NODE) --ip $(IP) $(NODE_ARGS)
	docker rm -f kinova_gen3_real_desc >/dev/null
```

In the header comment, line 8, change `#   make real IP=...    build KORTEX-enabled and run against the arm (attended)` to `#   make real IP=... MOUNT=...  build KORTEX-enabled and run against the arm (attended)`.

`deploy/compose.yaml`: in `kinova_gen3_driver.command`, delete the two lines

```yaml
      - --urdf
      - /module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf
```

and before the `# Owned by rammp-org/RAMMP-CuRobo.` comment of `curobo_planner`, add:

```yaml
  # The arm's model and TF. The driver reads /arm/robot_description at startup and
  # exits without it, so this is selected together with kinova_gen3_driver.
  # RAMMP_ROBOT_DIR is the robot's config directory (rammp-deployments:
  # december_2026/config/robots/current). Unset, the volume spec is invalid and docker
  # refuses to start the container -- loud, never a silently level arm.
  arm_state_publisher:
    image: kinova-gen3-ros2:kortex
    network_mode: host
    ipc: host
    volumes:
      - ${RAMMP_ROBOT_DIR}:/config/robot:ro
    command:
      - ros2
      - launch
      - kinova_gen3_description
      - description.launch.py
      - mount_file:=/config/robot/mount.yaml

```

Create `rammp-alternative.description.yaml`:

```yaml
# The arm's model and TF: robot_state_publisher on the model rendered from the
# robot's mount file.
#
# The third container of this module, same image as the driver. The driver reads
# /arm/robot_description at startup and cannot run without it, so select this with
# kinova_gen3_driver. Defined in deploy/compose.yaml for the same reason as the
# others: it is the only path sheppy interpolates ${VAR} on, and the mount
# directory has to come from the host.
id: kinova_gen3_description
tier: experimental
kind: docker
compose:
  file: deploy/compose.yaml
  service: arm_state_publisher
publishes:
  - /arm/robot_description
  - /tf
  - /tf_static
subscribes:
  - /joint_states
```

- [ ] **Step 4: Run it to verify it passes**

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'cd ~/mount_ws/src/kinova_gen3_ros2 && make build CORE_REF=feat/tilted-arm-mount && ROS_DOMAIN_ID=79 make smoke'
```

Expected: the last line is `PASS: matched 'kinova_gen3_node up' in logs and exited on SIGTERM in <n>s`.

```bash
ssh abra 'cd ~/mount_ws/src/kinova_gen3_ros2 && make e2e RT_CORE=-1 2>&1 | tail -4'
```

Expected: both `send_trajectory.py` calls exit 0, and the scheduling line reads `SCHED_FIFO` priority 80. `RT_CORE=-1` keeps the test off abra's isolated core.

From the Mac:

```bash
cd /Users/swapnil/atdev/kinova-gen3-ros2-mount && make check
```

Expected: one `ok` line for each fragment, including `  rammp-alternative.description.yaml: ok`.

- [ ] **Step 5: Check that the image holds no model file**

```bash
ssh abra "docker run --rm --entrypoint bash kinova-gen3-ros2:humble -c 'find /module_ws/install -name \"*.urdf\" | wc -l'"
```

Expected: `0`.

- [ ] **Step 6: Check SIGTERM while the node is waiting (Review Focus 5)**

```bash
ssh abra 'docker run -d --name wait-term --network host -e ROS_DOMAIN_ID=79 kinova-gen3-ros2:humble >/dev/null; sleep 3; docker stop -t 5 wait-term >/dev/null; docker logs wait-term 2>&1 | grep -c FATAL; docker rm wait-term >/dev/null'
```

Expected: `0`. The node stops within the grace period and does not log the "no model" FATAL.

- [ ] **Step 7: Commit**

```bash
git add Dockerfile Makefile scripts/smoke.sh deploy/compose.yaml rammp-alternative.description.yaml
git commit -m "build: ship kinova_gen3_description; start arm_state_publisher before the node

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 7: Client scripts and the conformance suite use the new names

**Files:**

- Modify: `kinova_gen3_ros2/test/send_goto_pose.py`, `send_goto_pose_sequence.py`, `send_goto_pose_tour.py`, `scenario_carry_level.py`, `sweep_constraints.py` (every `base_link`)
- Modify: `kinova_gen3_ros2/test/probe_reachable.py:47`
- Modify: `kinova_gen3_ros2/test/conformance/checks_state.py:18`, `kinova_gen3_ros2/test/conformance/checks_gripper.py:19`
- Modify: `kinova_gen3_ros2/test/conformance/README.md:9-10`

**Interfaces:**

- Consumes: the `/joint_states` names (Task 4) and the `arm_mount` frame (Task 5).

- Produces: nothing new.

- [ ] **Step 1: Run the conformance state checks to verify they fail**

Start the sim stack on abra and run the `state` and `gripper` sections:

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && export ROS_DOMAIN_ID=79 && \
  (ros2 launch kinova_gen3_description bringup.launch.py sim:=true \
     mount_file:=\$HOME/mount_ws/src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml > /tmp/bringup.log 2>&1 &) && \
  sleep 12 && cd ~/mount_ws/src/kinova_gen3_ros2/kinova_gen3_ros2/test/conformance && \
  python3 run_conformance.py --sections state,gripper 2>&1 | tail -6; \
  pkill -INT -f \"ros2 launch kinova_gen3_description\"'"
```

Expected: FAIL on `/joint_states publishes seven named arm joints`, with `names are ['arm_joint_1', …], expected ['joint_1', …]`, and on the gripper checks that look up `robotiq_85_left_knuckle_joint`. (If `run_conformance.py` rejects the comma form, run it twice: `--sections state` and `--sections gripper`. `--list` shows the section names.)

- [ ] **Step 2: Update the names**

```bash
cd /Users/swapnil/atdev/kinova-gen3-ros2-mount/kinova_gen3_ros2/test
sed -i.bak 's/base_link/arm_mount/g' send_goto_pose.py send_goto_pose_sequence.py \
  send_goto_pose_tour.py scenario_carry_level.py sweep_constraints.py
sed -i.bak 's/ARM_JOINTS = \[f"joint_{i}"/ARM_JOINTS = [f"arm_joint_{i}"/' probe_reachable.py
sed -i.bak 's/robotiq_85_left_knuckle_joint\. The planner/arm_robotiq_85_left_knuckle_joint. The planner/' probe_reachable.py
sed -i.bak 's/expected = \[f"joint_{i}"/expected = [f"arm_joint_{i}"/' conformance/checks_state.py
sed -i.bak 's/^KNUCKLE = "robotiq_85_left_knuckle_joint"/KNUCKLE = "arm_robotiq_85_left_knuckle_joint"/' conformance/checks_gripper.py
rm -f *.bak conformance/*.bak
grep -rn 'base_link\|"joint_{i}"\|f"joint_\|^KNUCKLE' *.py conformance/*.py
```

Expected: only `conformance/checks_gripper.py:19:KNUCKLE = "arm_robotiq_85_left_knuckle_joint"`.

`kinova_gen3_ros2/test/conformance/README.md` lines 9–10 become:

```markdown
# 1. start the node and its model yourself, with the mode you want to test
ros2 launch kinova_gen3_description bringup.launch.py sim:=true \
  mount_file:=src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml \
  arbitration_mode:=enforced
```

- [ ] **Step 3: Run the conformance suite to verify it passes**

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra "bash -c 'source /opt/ros/humble/setup.bash && source ~/mount_ws/install/setup.bash && export ROS_DOMAIN_ID=79 && \
  (ros2 launch kinova_gen3_description bringup.launch.py sim:=true arbitration_mode:=enforced \
     mount_file:=\$HOME/mount_ws/src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml > /tmp/bringup.log 2>&1 &) && \
  sleep 12 && cd ~/mount_ws/src/kinova_gen3_ros2/kinova_gen3_ros2/test/conformance && \
  python3 run_conformance.py 2>&1 | tail -3; \
  pkill -INT -f \"ros2 launch kinova_gen3_description\"'"
```

Expected: `32 passed, 0 failed, 0 skipped (mode=enforced)`, the bar in CONTRIBUTING. This is sim: the `motion` section drives `SimTransport` only.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_ros2/test
git commit -m "test: client scripts and conformance speak arm_mount and arm_ joint names

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 8: Docs and changelog

**Files:**

- Modify: `README.md:122` (joint_states row), `:303-371` (Robot model and TF), `:379` (`--urdf` row), `:407`, `:562-568`, `:618`
- Modify: `docs/guide-goto-ee-pose.md:9, 51, 63-64, 91`, `docs/guide-goto-actions.md:8, 97`
- Modify: `docs/getting-started.mdx:159-162, 258-260`, `docs/on-robot-runbook.md:48-49`
- Modify: `CHANGELOG.md:3` (Unreleased)

**Interfaces:**

- Consumes: everything above. Produces: nothing.

- [ ] **Step 1: Write the failing check**

```bash
cd /Users/swapnil/atdev/kinova-gen3-ros2-mount
grep -rn -- '--urdf\|kinova_gen3_7dof\|expand_model\|`joint_1`\|articulated:=' README.md docs kinova_gen3_ros2/test/conformance/README.md
grep -rn 'base_link' docs/guide-goto-ee-pose.md docs/guide-goto-actions.md
```

Expected: matches. Each one is a stale statement this task removes.

- [ ] **Step 2: Rewrite the stale passages**

`README.md` line 122, the `joint_states` row: replace `` | `joint_1`..`joint_7`; `position`/`velocity`/`effort` all filled. `` with `` | `arm_joint_1`..`arm_joint_7` (names from the model), then `arm_robotiq_85_left_knuckle_joint` when a gripper sink is wired; `position`/`velocity`/`effort` all filled. `` Leave the rest of the row as it is.

`README.md` lines 303–371, from `## Robot model and TF` up to but not including `### Known gaps`, become:

````markdown
## Robot model and TF

`kinova_gen3_description` holds the robot model and the launch plumbing. Every robot
declares its arm mount in `mount.yaml`: `robot: <name>` and `rpy_deg: [r, p, y]`, the
orientation of `arm_base_link` in `arm_mount`, in degrees. In rammp-deployments that is
`config/robots/<robot>/mount.yaml`, and containers see it at `/config/robot/mount.yaml`.
`mount_file` has **no default**. A default would silently render a level arm on a
tilted one, and gravity compensation would then be wrong by the tilt.

```bash
ros2 launch kinova_gen3_description bringup.launch.py sim:=true mount_file:=<robot>/mount.yaml
# TF and the model only, without owning the arm:
ros2 launch kinova_gen3_description description.launch.py mount_file:=<robot>/mount.yaml
```

The first line the description logs is the active robot:
`embodiment: flatbot (mount rpy_deg [0.0, 30.0, 0.0])`.

### One model, rendered at launch

`description.launch.py` renders `urdf/kinova_gen3.urdf.xacro` with the mount file, using
`kinova_gen3_render.py` and stripping `kortex_description`'s `<ros2_control>` block. It
then runs `robot_state_publisher` with `robot_description` remapped to
**`/arm/robot_description`** (latched: RELIABLE + TRANSIENT_LOCAL). `/tf` and
`/joint_states` stay global. The driver reads that topic once at startup and builds its
dynamics from it, and cuRobo does the same, so the driver, TF and the planner cannot
disagree. Nothing is rendered at build time, and the image holds no URDF file.

- Root `arm_mount`: Z against gravity when the chair is level, X forward, origin at the
  arm's base. All Cartesian I/O is in this frame: `/ee_state`, `GoToEEPose`,
  `/setpoint/pose`, `/setpoint/twist`. On dojo the mount is identity and every number
  is unchanged. On flatbot and gen_1_5 it is +30° about Y.
- Every name carries the `arm_` prefix (`arm_joint_1`…`arm_joint_7`, `arm_base_link`, …),
  because the chair owns `base_link` (REP-105).
- `arm_tool_frame` sits 0.120 m along `arm_end_effector_link`'s +Z. That is cuRobo
  v0.7.8's `tool_frame`, where every `PlanToPose` goal is positioned.
- The model has 13 DOF. The driver reduces it to the seven arm joints itself,
  locking the gripper at q = 0, which keeps the gripper's mass at the wrist.
  `robot_state_publisher` derives the five mimic joints from
  `arm_robotiq_85_left_knuckle_joint`, which the driver publishes.

Bench (the driver's standalone apps take `--urdf`):

```bash
ros2 run kinova_gen3_description kinova_gen3_render.py --mount <robot>/mount.yaml --out /tmp/arm.urdf
```

### The invariant

**`/joint_states` names come from the model**, via `Dynamics::joint_names()` plus the
gripper joint, which is checked to exist at startup. Before, names were hard-coded,
and once they disagreed with the URDF (`joint_1` vs `gen3_joint_1`).
`robot_state_publisher` matched nothing and held TF still while the arm moved, and
nothing errored. `test_tf_updates.py` asserts that the transform *changes*.

`--ee-frame` (default `arm_end_effector_link`) is resolved by name in `Dynamics`,
which throws if the frame is absent.
````

`README.md` line 379: replace the `--urdf` row with:

```markdown
| `--ee-frame <frame>`      | `arm_end_effector_link`      | Frame `Dynamics` (and so `/ee_state`) uses as the end effector. The model itself comes from `/arm/robot_description`, which the node waits for (30 s) at startup.                    |
```

`README.md` line 407: change `ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf \` to `ros2 run kinova_gen3_ros2 kinova_gen3_node --sim \`.

`README.md` lines 562–568: change the bringup line to
`ros2 launch kinova_gen3_description bringup.launch.py sim:=true mount_file:=src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml`.
Change the "just the node" pair to:

```sh
# or the node alone; it waits for /arm/robot_description, so start the description too:
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim
```

On the real-arm line, add `mount_file:=<rammp-deployments>/december_2026/config/robots/current/mount.yaml`.

`README.md` line 618: change `ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf` to `ros2 launch kinova_gen3_description bringup.launch.py sim:=true mount_file:=src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml`.

`docs/guide-goto-ee-pose.md`:

```bash
sed -i.bak 's/base_link/arm_mount/g' docs/guide-goto-ee-pose.md docs/guide-goto-actions.md && \
  rm docs/guide-goto-ee-pose.md.bak docs/guide-goto-actions.md.bak
```

Then line 9 of `docs/guide-goto-ee-pose.md` reads `` …to a target pose in `arm_mount`. `` Append this sentence to that line: ``  `arm_mount` is gravity-aligned when the chair is level; on dojo it coincides with the arm's base, and on flatbot and gen_1_5 the arm's base is tilted 30° forward in it. A goal in `base_link` is rejected (the reason is in the node log). ``

`docs/getting-started.mdx` lines 159–162: delete the `--urdf` and model-path lines from the driver `command:`. After the driver service, add the same `arm_state_publisher` service as `deploy/compose.yaml` (Task 6), with `image: ghcr.io/rammp-org/kinova-gen3-ros2:1.1.0` (the version in this file; the release bump updates both) and a `volumes:` entry `- ./robot:/config/robot:ro`. Add one sentence before the compose block: `Put your robot's mount.yaml in ./robot/ (robot: <name>, rpy_deg: [r, p, y]); there is no default.` Lines 258–260 become:

```sh
source install/setup.bash
ros2 launch kinova_gen3_description bringup.launch.py sim:=true \
  mount_file:=src/kinova_gen3_ros2/kinova_gen3_description/test/mounts/level.yaml
```

`docs/on-robot-runbook.md` lines 48–49 become:

```sh
ros2 launch kinova_gen3_description bringup.launch.py sim:=false ip:=192.168.1.10 \
  mount_file:=<rammp-deployments>/december_2026/config/robots/current/mount.yaml
```

`CHANGELOG.md`: under `## Unreleased` (line 3), add:

```markdown

### Changed (breaking)

- **The arm's model is rendered at launch from the robot's `mount.yaml` and published
  on `/arm/robot_description`** (latched). `description.launch.py` and
  `bringup.launch.py` require `mount_file` and have no default. The build no longer
  produces `kinova_gen3.urdf` or `kinova_gen3_7dof.urdf`, and `expand_model.py` is gone.
  `kinova_gen3_render.py --mount --out` renders one for the bench.
- **Every arm name carries `arm_`, and the root is `arm_mount`** (Z against gravity when
  the chair is level). `/joint_states` carries `arm_joint_1`…`arm_joint_7` and
  `arm_robotiq_85_left_knuckle_joint`, taken from the model. New frame
  `arm_tool_frame`: cuRobo's `tool_frame`, 0.120 m beyond the flange.
- **`kinova_gen3_node --urdf` is removed.** The node reads `/arm/robot_description`
  before the RT loop starts (30 s timeout, error names `arm_state_publisher`). Core
  reduces the model to the arm. `--ee-frame` defaults to `arm_end_effector_link`.
- **`GoToEEPose` requires `frame_id: arm_mount`.** `base_link` is rejected, and the
  reason is in the node log. On flatbot and gen_1_5, Cartesian numbers change by the
  30° mount (intended). Re-record saved poses there.
- The image now contains `kinova_gen3_description`, so `arm_state_publisher` runs from
  it. The released 1.1.0 image did not, although the deployment manifest already ran
  `description.launch.py` from it.
```

- [ ] **Step 3: Run the check to verify it passes**

```bash
cd /Users/swapnil/atdev/kinova-gen3-ros2-mount
grep -rn -- '--urdf\|kinova_gen3_7dof\|expand_model\|`joint_1`\|articulated:=' README.md docs kinova_gen3_ros2/test/conformance/README.md
grep -rn 'base_link' docs/guide-goto-ee-pose.md docs/guide-goto-actions.md
make lint
```

Expected: the first grep prints only README lines that mention `--urdf` for the driver's *standalone apps* (the bench note). The second prints only the guide sentence saying `base_link` is rejected. `make lint` passes.

- [ ] **Step 4: Commit**

```bash
git add README.md docs CHANGELOG.md
git commit -m "docs: arm_mount, mount_file, /arm/robot_description; --urdf removed

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

### Task 9: Pin the driver release that lands the `UrdfXml` constructor

This is its own step because the tag does not exist when this plan is written. Until it does, the PR's CI builds the companion branch `feat/tilted-arm-mount` of kinova-gen3-driver (`.github/workflows/build.yml`, "Resolve sibling refs"), and abra builds from the rsynced driver worktree. The informational `smoke-released` leg is red until this task lands, by design. It was already red on `dev`, which compiles against driver work past `v1.2.0`.

**Files:**

- Modify: `kinova_gen3.repos:34-36`

**Interfaces:**

- Consumes: the kinova-gen3-driver commit/tag that lands the `UrdfXml` constructor.

- Produces: a reproducible released-pin build.

- [ ] **Step 1: Find the tag and check that it carries the API**

```bash
gh release list -R rammp-org/kinova-gen3-driver --limit 5
TAG=<the newest tag from that list whose notes include the UrdfXml constructor>
gh api "repos/rammp-org/kinova-gen3-driver/contents/include/kinova_lowlevel/dynamics.h?ref=$TAG" \
  -H "Accept: application/vnd.github.raw" | grep -n "struct UrdfXml\|joint_names"
```

Expected: two lines, `struct UrdfXml { std::string xml; };` and `std::vector<std::string> joint_names() const;`. If either is missing, the tag is wrong or not yet cut. Stop.

- [ ] **Step 2: Bump the pin**

Replace `kinova_gen3.repos` lines 34–36:

```yaml
    # v1.2.0: TrajectoryGoal::speed_scale, kMinSpeedScale and
    # CommandSink::on_set_speed_override, which this branch compiles against.
    version: v1.2.0
```

with (substituting the tag found in Step 1, e.g. `v1.4.0`):

```yaml
    # <TAG>: Dynamics(UrdfXml, ee_frame) with reduction to the arm chain and
    # Dynamics::joint_names() -- the node builds its dynamics and /joint_states
    # names from /arm/robot_description -- plus everything v1.2.0 brought.
    version: <TAG>
```

- [ ] **Step 3: Verify the released-pin build**

```bash
rsync -a --delete --exclude .git --exclude build --exclude install --exclude log \
  /Users/swapnil/atdev/kinova-gen3-ros2-mount/ abra:mount_ws/src/kinova_gen3_ros2/
ssh abra 'cd ~/mount_ws/src/kinova_gen3_ros2 && make build && ROS_DOMAIN_ID=79 make smoke'
```

Expected: `PASS: matched 'kinova_gen3_node up' in logs and exited on SIGTERM in <n>s`. Here `make build` has no `CORE_REF`, so it uses the pin.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3.repos
git commit -m "chore: pin kinova-gen3-driver <TAG> (UrdfXml constructor, joint_names)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

______________________________________________________________________

## Self-Review

**Spec coverage (kinova-gen3-ros2 section):**

- Root `world` → `arm_mount`, `prefix` always `arm_`, required `mount_file` via `xacro.load_yaml` and `radians()`: Task 1.
- `arm_tool_frame` at cuRobo's transform: Task 1. Pinned against cuRobo v0.7.8's numbers by composing bracelet → end_effector → tool_frame.
- Build-time expansion removed, render at launch, `ros2_control` stripped, required `mount_file`, embodiment log line: Tasks 1–2. "No flat model in the image" is checked in Task 6 Step 5.
- RSP remapped to `/arm/robot_description`, `/tf` and `/joint_states` global: Task 2.
- Bench render script `--mount --out`: Task 1.
- Driver node drops `--urdf`, reads the topic before the RT loop with a timeout and an error naming `arm_state_publisher`, `UrdfXml` Dynamics, `--ee-frame arm_end_effector_link`: Tasks 3–4.
- `/joint_states` names from the model plus the gripper joint, checked at startup, hard-coded list removed: Tasks 3–4.
- `GoToEEPose` requires `arm_mount`, exact `base_link` text: Task 5.
- "What changes for consumers": `/joint_states` name readers in this repo (conformance, `probe_reachable.py`) and `GoToEEPose` callers (client scripts): Task 7. Docs and changelog: Task 8.
- Rollout step 1 (dojo no-op): `test_level_mount_forward_kinematics_matches_old_model` (Task 1) is the offline form. On-hardware checks belong to the rollout, not this plan; the arms are off.
- Manifest changes belong to rammp-deployments. This repo's own compose and fragments get the matching edit (Task 6) so that its module definition still starts.
- Driver pin bump as its own step: Task 9.

**Placeholder scan:** the only deliberate unknown is `<TAG>` in Task 9. That release does not exist yet, and Step 1 gives the exact procedure to find and verify it. Two README edits in Task 8 are given as "replace this text with that" rather than as a whole block, because the surrounding table rows are 300+ characters wide. Their exact old and new text is stated.

**Type consistency:** `wait_for_robot_description(const rclcpp::Node::SharedPtr&, std::chrono::milliseconds)` is called with `std::chrono::seconds` (implicit conversion) in Task 4. `joint_state_names(const std::string&, std::vector<std::string>)` takes `dyn.joint_names()` by value. `Ros2Backend(node, std::vector<std::string>)` is used the same way in bringup and in both tests. `kRobotDescriptionTopic` and `kGripperJoint` are used with the `kinova_gen3_ros2::` qualifier in `bringup_node.cpp`. `ee_pose_frame_rejection` sits in namespace `kinova_gen3_ros2`, visible unqualified in `goto_ee_pose_server.h` and in `message_mapping_test.cpp` (`using namespace kinova_gen3_ros2;`). `render` / `load_mount` / `embodiment_line` have the same names in Task 1 and Task 2.

**Review Focus:** all five have a test or an explicit check in their owning task: Task 3 tests 1 and 3, Task 1 tests 2, Task 5 tests 4, and Task 6 Step 6 checks 5.
