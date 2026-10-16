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


def test_phase_marker_is_not_a_tap(tmp_path):
    """A marker of 2 is the baseline/tap boundary, never a tap."""
    from analyze import load_csv
    from synthetic import sweep, write_csv

    model = ArmModel(URDF)
    t, q, qd, tau, _, push = sweep(model, duration=4.0)
    path = tmp_path / "r.csv"
    write_csv(path, t, q, qd, tau, push, phase_at=2.0)
    _, _, _, _, taps, phase = load_csv(path)
    assert phase == 2.0
    assert all(abs(m - 2.0) > 1e-6 for m in taps)


def test_baseline_map_removes_pose_dependent_bias():
    """A bias that is a function of pose survives the observer; the baseline map removes it."""
    from analyze import baseline_map

    model = ArmModel(URDF)
    t, q, qd, tau, _, _push = sweep(model, duration=6.0)
    bias = 1.5 * np.sin(q[:, 3])[:, None] * np.array([0, 1, 0, 1, 0, 0, 0])
    r = run_observer(model, t, q, qd, tau - bias, gain=20.0)
    # The sweep has a 1.5 s period; a 2 s baseline covers every pose once with no
    # push in it (the first push starts at 2.4 s). The window after it skips the
    # pushes and the observer's decay tail behind each one.
    phase = 2.0
    fixed, matched = baseline_map(t, q, qd, r, phase)
    after = (t >= 3.0) & (t < 5.0) & matched
    assert matched[t >= phase].mean() > 0.9
    assert np.abs(fixed[after, 3]).max() < 0.4
    assert np.abs(r[after, 3]).max() > 1.0
