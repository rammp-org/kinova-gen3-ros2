#!/usr/bin/env python3
"""Compare the generated model against the hand-edited one it replaces.

Gravity compensation, cuRobo, the in-loop IK and the DLS twist solve all read these
numbers, and RViz looking correct proves nothing about any of them. So the swap is
gated here rather than by eye.

Two differences are KNOWN and deliberate; everything else must match exactly.

  1. The gripper articulates. The old model froze every Robotiq joint to type="fixed",
     so Pinocchio lumped the whole gripper into joint_7 and the model had 7 movable
     joints. The generated one has 13. That is the point of the change.

  2. 0.436 kg is missing at the wrist -- a RealSense D415 (0.072 kg) and a 0.5 kg
     custom camera mount that the c3pzero author added and upstream cannot know about.
     Deferred by decision, and PINNED by test_known_wrist_mass_delta below so it cannot
     drift or be forgotten: add the camera back and that test fails, telling you to.

Everything about the arm proper -- link masses, kinematics -- must be identical, and is.
"""

import os

import numpy as np
import pinocchio as pin
import pytest

NEW = os.environ["KINOVA_NEW_URDF"]  # articulated gripper, 13 DOF, for RViz/TF
SEVEN = os.environ["KINOVA_7DOF_URDF"]  # gripper frozen, 7 DOF, for the driver
OLD = os.environ["KINOVA_OLD_URDF"]
OLD_EE, NEW_EE = "gen3_end_effector_link", "end_effector_link"
ARM = [f"joint_{i}" for i in range(1, 8)]
TOL = 1e-9
# The wrist camera + mount the generated model omits. Pinned deliberately: see the
# module docstring.
KNOWN_WRIST_MASS_DELTA_KG = 0.436
N_SAMPLES = 500


def _load(path):
    model = pin.buildModelFromUrdf(path)
    return model, model.createData()


@pytest.fixture(scope="module")
def models():
    """Compare the old model against the 7-DOF variant, not the articulated one.

    Both have the gripper lumped into the wrist, so this is a like-for-like comparison
    of the thing the DRIVER loads. The articulated model is checked separately for DOF
    only -- Pinocchio would not accept a shared q vector across 7 and 13 DOF anyway."""
    return _load(OLD), _load(SEVEN)


def _set_arm(model, q, angles, prefix=""):
    """Write the seven arm angles into q by JOINT NAME.

    Not by slicing: the two models have different nq (11 vs 21) and different joint
    counts, so any assumption about layout would compare different joints to each other
    and quietly report agreement.
    """
    for name, a in zip(ARM, angles):
        jid = model.getJointId(prefix + name)
        idx = model.joints[jid].idx_q
        if model.joints[jid].nq == 2:  # continuous joint, packed (cos, sin)
            q[idx], q[idx + 1] = np.cos(a), np.sin(a)
        else:
            q[idx] = a
    return q


def _arm_configs(n, seed=0):
    rng = np.random.default_rng(seed)
    return [rng.uniform(-np.pi, np.pi, len(ARM)) for _ in range(n)]


def test_arm_link_masses_are_identical(models):
    """The invariant that matters: the arm proper is the same robot."""
    (old_m, _), (new_m, _) = models
    for name in ARM[:-1]:  # joint_7 carries the wrist delta; see below
        o = old_m.inertias[old_m.getJointId("gen3_" + name)].mass
        n = new_m.inertias[new_m.getJointId(name)].mass
        assert abs(o - n) < TOL, f"{name} body mass {o:.6f} != {n:.6f} kg"


def test_forward_kinematics_matches(models):
    """Mass differences do not move frames. If FK disagrees, the two models are not the
    same kinematic chain and nothing else in this file matters."""
    (old_m, old_d), (new_m, new_d) = models
    old_id, new_id = old_m.getFrameId(OLD_EE), new_m.getFrameId(NEW_EE)
    worst = 0.0
    for angles in _arm_configs(N_SAMPLES):
        q_old = _set_arm(old_m, pin.neutral(old_m), angles, prefix="gen3_")
        q_new = _set_arm(new_m, pin.neutral(new_m), angles)
        pin.forwardKinematics(old_m, old_d, q_old)
        pin.updateFramePlacement(old_m, old_d, old_id)
        pin.forwardKinematics(new_m, new_d, q_new)
        pin.updateFramePlacement(new_m, new_d, new_id)
        worst = max(
            worst,
            float(
                np.linalg.norm(
                    old_d.oMf[old_id].translation - new_d.oMf[new_id].translation
                )
            ),
        )
    assert worst < TOL, f"EE position differs by up to {worst:.6e} m"


def test_known_wrist_mass_delta(models):
    """Pin the deferred camera, so it cannot be silently forgotten.

    If someone adds the wrist camera and mount back to the xacro, this fails and says
    so -- which is the point. Update the constant then, do not widen it.
    """
    (old_m, _), (new_m, _) = models
    delta = sum(i.mass for i in old_m.inertias) - sum(i.mass for i in new_m.inertias)
    assert abs(delta - KNOWN_WRIST_MASS_DELTA_KG) < 1e-3, (
        f"total-mass delta is {delta:.6f} kg, expected {KNOWN_WRIST_MASS_DELTA_KG:.6f} "
        "(the omitted wrist camera + mount). If you added the camera back, set "
        "KNOWN_WRIST_MASS_DELTA_KG to 0.0; if something else moved, find out what."
    )


def test_driver_model_is_seven_dof(models):
    """Core's Dynamics asserts nv == kNumJoints == 7 and aborts otherwise:
        Dynamics: URDF nv=13 != kNumJoints=7 (wrong URDF for this build)
    JointVec is a fixed-size 7-vector, so this is not negotiable."""
    (old_m, _), (seven_m, _) = models
    assert old_m.nv == 7, f"old model unexpectedly has {old_m.nv} DOF"
    assert (
        seven_m.nv == 7
    ), f"the driver's model has {seven_m.nv} DOF; core's Dynamics will refuse it"


def test_rviz_model_is_articulated():
    """...while robot_state_publisher needs the fingers to MOVE, which is the whole
    reason two models are generated from one xacro."""
    art = pin.buildModelFromUrdf(NEW)
    assert art.nv > 7, "gripper is frozen in the articulated model; RViz cannot move it"
    assert art.existJointName(
        "robotiq_85_left_knuckle_joint"
    ), "the actuated gripper joint is missing"


def test_freezing_the_gripper_preserves_its_mass():
    """Freezing must drop DEGREES OF FREEDOM, not mass. Pinocchio lumps a fixed joint's
    body into its parent, so the wrist still carries the gripper -- which is what
    gravity compensation depends on."""
    art = pin.buildModelFromUrdf(NEW)
    seven = pin.buildModelFromUrdf(SEVEN)
    a, s = sum(i.mass for i in art.inertias), sum(i.mass for i in seven.inertias)
    assert abs(a - s) < TOL, f"freezing changed total mass: {a:.6f} vs {s:.6f} kg"
