"""Does the observer recover an injected external torque while the arm moves?

Synthetic: a 7-DOF Gen3 model (any URDF with *joint_1..7; the driver's
models/gen3_7dof_2f85.urdf works) is driven through a smooth joint-space sweep.
The "measured" torque is what the joint must supply for that motion, minus an
external push injected on one joint for 0.3 s. Both estimators see the same
q, qd, tau at 50 Hz, the rate /joint_states actually runs at.

Set KINOVA_URDF to a URDF path. Without it the test is skipped.
"""

import os

import numpy as np
import pytest

from observer import ArmModel, quasi_static_residual, run_observer

URDF = os.environ.get("KINOVA_URDF")
pytestmark = pytest.mark.skipif(URDF is None, reason="KINOVA_URDF not set")

from synthetic import PUSH_JOINT, PUSH_NM, RATE, sweep


def test_observer_tracks_push_in_motion():
    model = ArmModel(URDF)
    t, q, qd, tau, _, push = sweep(model, duration=6.0)
    r = run_observer(model, t, q, qd, tau, gain=10.0)
    qs = np.array([quasi_static_residual(model, q[k], tau[k]) for k in range(len(t))])

    moving = np.abs(qd).max(axis=1) > 0.05
    quiet = moving & ~push & (t > 0.5) & ((t < 2.0) | ((t > 3.6) & (t < 4.8)))

    # The pulse: observer reaches most of it within the 0.3 s window.
    assert r[push, PUSH_JOINT].max() > 0.8 * PUSH_NM
    # Away from the pulse, with the arm in motion, the observer sits near zero
    # while the quasi-static residual carries the arm's own inertia.
    obs_rms = np.sqrt(np.mean(r[quiet] ** 2))
    qs_rms = np.sqrt(np.mean(qs[quiet] ** 2))
    assert obs_rms < 0.3
    assert qs_rms > 3 * obs_rms


def test_both_agree_at_rest():
    model = ArmModel(URDF)
    q = np.array([0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57])
    tau = model.gravity(q) - np.array(
        [0, 0, 0, 2.0, 0, 0, 0]
    )  # 2 N.m push on the elbow
    t = np.arange(0.0, 2.0, 1.0 / RATE)
    n = len(t)
    r = run_observer(
        model, t, np.tile(q, (n, 1)), np.zeros((n, 7)), np.tile(tau, (n, 1))
    )
    qs = quasi_static_residual(model, q, tau)
    assert np.allclose(r[-1], qs, atol=0.05)
    assert abs(qs[3] - 2.0) < 1e-9


def test_sign_detection_from_still_samples():
    from analyze import detect_tau_sign

    model = ArmModel(URDF)
    q = np.array([0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57])
    n = 40
    g = np.tile(model.gravity(q), (n, 1))
    assert detect_tau_sign(model, np.tile(q, (n, 1)), np.zeros((n, 7)), g) == 1.0
    assert detect_tau_sign(model, np.tile(q, (n, 1)), np.zeros((n, 7)), -g) == -1.0
