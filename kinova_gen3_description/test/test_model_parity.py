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
