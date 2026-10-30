"""Collision detection and payload weighing, on synthetic runs with a known truth.

The "measured" torque is what the joints must supply for the motion with the
true payload, plus a pose-dependent model error like the one on abra
(kinova-gen3-driver#5), plus sensor noise, minus any external torque. The
monitor only ever sees the bare model.

Set KINOVA_URDF to a URDF path. Without it the test is skipped.
"""

import os

import numpy as np
import pytest

from contact import (
    PROFILES,
    ContactMonitor,
    Floor,
    Payload,
    Profile,
    calibrate_floor,
    fit_payload,
    grasp_point,
    payload_regressor,
    point_jacobian,
    with_payload,
)
from observer import ArmModel

URDF = os.environ.get("KINOVA_URDF")
pytestmark = pytest.mark.skipif(URDF is None, reason="KINOVA_URDF not set")

RATE = 100.0
Q0 = np.array([0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57])
LIFT = np.array([0.0, -0.06, 0.0, 0.08, 0.0, -0.02, 0.0])
BOTTLE = 0.5  # kg
NOISE = 0.05  # N.m RMS


@pytest.fixture(scope="module")
def model():
    return ArmModel(URDF)


def minjerk(t, t0, t1, a, b):
    """q, qd, qdd of a min-jerk move from a to b over [t0, t1], held outside it."""
    T = t1 - t0
    s = np.clip((t - t0) / T, 0.0, 1.0)[:, None]
    inside = ((t >= t0) & (t <= t1))[:, None]
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) / T * inside
    acc = (60 * s - 180 * s**2 + 120 * s**3) / T**2 * inside
    d = b - a
    return a + d * p, d * v, d * acc


def carry(t, t0, q_start):
    """A brisk sweep on 2, 4, 6 and the wrist roll, starting at t0 from q_start."""
    amp = np.array([0.0, 0.3, 0.0, 0.4, 0.0, 0.4, 0.8])
    w = 2 * np.pi / 2.0
    s = np.where(t >= t0, t - t0, 0.0)[:, None]
    on = (t >= t0)[:, None]
    q = q_start + amp * (1 - np.cos(w * s)) / 2
    qd = amp * w * np.sin(w * s) / 2 * on
    qdd = amp * w * w * np.cos(w * s) / 2 * on
    return q, qd, qdd


def bias(q):
    """Pose-dependent model error, the shape of the one measured on abra."""
    return np.column_stack(
        [
            np.zeros(len(q)),
            1.5 * np.sin(q[:, 3]) + 4.0,
            np.zeros(len(q)),
            1.0 * np.sin(q[:, 1]),
            np.zeros(len(q)),
            np.zeros(len(q)),
            np.zeros(len(q)),
        ]
    )


def measured_tau(model, q, qd, qdd, load, ext, seed=0):
    """tau = ID(true payload) + bias - external, with load in [0, 1] scaling the bottle in."""
    held = with_payload(model, Payload(BOTTLE, tuple(grasp_point(model))))
    bare = np.array(
        [model.inverse_dynamics(q[k], qd[k], qdd[k]) for k in range(len(q))]
    )
    full = np.array([held.inverse_dynamics(q[k], qd[k], qdd[k]) for k in range(len(q))])
    tau = bare + load[:, None] * (full - bare) + bias(q) - ext
    return tau + np.random.default_rng(seed).normal(0.0, NOISE, tau.shape)


def force_at_grasp(model, q, mask, force):
    """External joint torque from a world-frame force on the held object."""
    c = grasp_point(model)
    ext = np.zeros_like(q)
    for k in np.flatnonzero(mask):
        ext[k] = point_jacobian(model, q[k], c).T @ force
    return ext


def pick_and_carry(model, collision=None, duration=9.0):
    """Still at Q0, lift the bottle at 1.5 to 2.0 s, still, then carry from 2.8 s.

    The bottle's weight comes onto the gripper as it leaves the table, over 0.1 s
    early in the lift. collision: (t0, t1, world force) on the bottle.
    """
    t = np.arange(0.0, duration, 1.0 / RATE)
    q, qd, qdd = minjerk(t, 1.5, 2.0, Q0, Q0 + LIFT)
    cq, cqd, cqdd = carry(t, 2.8, Q0 + LIFT)
    after = t >= 2.8
    q[after], qd[after], qdd[after] = cq[after], cqd[after], cqdd[after]
    load = np.clip((t - 1.55) / 0.1, 0.0, 1.0)
    ext = np.zeros_like(q)
    if collision:
        a, b, f = collision
        ext = force_at_grasp(model, q, (t >= a) & (t < b), np.asarray(f, float))
    return t, q, qd, measured_tau(model, q, qd, qdd, load, ext)


@pytest.fixture(scope="module")
def floor(model):
    """Calibrated the way it would be on the arm: one hands-off carry, empty gripper."""
    t = np.arange(0.0, 8.0, 1.0 / RATE)
    q, qd, qdd = carry(t, 2.0, Q0 + LIFT)
    tau = measured_tau(model, q, qd, qdd, np.zeros(len(t)), np.zeros_like(q), seed=1)
    return calibrate_floor(model, t, q, qd, tau)


def run(mon, t, q, qd, tau, hooks=None, record=None):
    """Step the monitor through a run; hooks maps a time to a callable on the monitor."""
    hooks = dict(hooks or {})
    out = {}
    for k in range(len(t)):
        for at in [a for a in hooks if t[k] >= a]:
            out[at] = hooks.pop(at)(mon)
        mon.step(t[k], q[k], qd[k], tau[k])
        if record:
            record(mon, k)
    return out


GRASP = {1.2: lambda m: m.begin_load_change(), 2.5: lambda m: m.end_load_change()}


def test_regressor_matches_payload_model(model):
    p = Payload(0.7, (0.01, -0.02, 0.12))
    held = with_payload(model, p)
    rng = np.random.default_rng(3)
    for _ in range(5):
        q = rng.uniform(-np.pi, np.pi, 7)
        dg = held.gravity(q) - model.gravity(q)
        assert np.allclose(dg, payload_regressor(model, q) @ p.theta, atol=1e-9)


def test_fit_payload_recovers_com_from_several_poses(model):
    p = Payload(0.6, (0.02, 0.0, 0.11))
    rng = np.random.default_rng(4)
    Ys, drs = [], []
    for _ in range(6):
        q = Q0 + rng.uniform(-0.8, 0.8, 7)
        Y = payload_regressor(model, q)
        Ys.append(Y)
        drs.append(-Y @ p.theta)
    theta, rms = fit_payload(Ys, drs)
    got = Payload.from_theta(theta)
    assert (
        abs(got.mass - 0.6) < 1e-6
        and np.allclose(got.com, p.com, atol=1e-6)
        and rms < 1e-9
    )


def test_weighs_the_bottle_through_the_model_error(model, floor):
    """The 4 N.m shoulder bias is bigger than the bottle; the tare cancels it."""
    t, q, qd, tau = pick_and_carry(model, duration=3.0)
    mon = ContactMonitor(model, floor=floor)
    est = run(mon, t, q, qd, tau, GRASP)[2.5]
    assert est.ok, est.reason
    assert abs(est.payload.mass - BOTTLE) < 0.05
    assert mon.payload.mass == est.payload.mass and not mon.in_load_change


def test_headline_pick_bottle_carry_and_catch_a_bump(model, floor):
    """Pick up a bottle, carry it at speed, a person bumps it: one event, the bump."""
    bump = (6.0, 6.15, [25.0, 0.0, 0.0])  # 25 N from the side for 150 ms
    t, q, qd, tau = pick_and_carry(model, collision=bump)
    mon = ContactMonitor(model, profile=PROFILES["bottle"], floor=floor)
    run(mon, t, q, qd, tau, GRASP)
    assert len(mon.events) == 1, [(e.t, e.joint, round(e.score, 1)) for e in mon.events]
    ev = mon.events[0]
    assert 6.0 <= ev.t <= 6.08 and ev.moving


def test_grasp_without_declaring_it_looks_like_a_collision(model, floor):
    """Why begin/end_load_change exist: the bottle leaving the table is a fast step."""
    t, q, qd, tau = pick_and_carry(model, duration=3.0)
    mon = ContactMonitor(model, profile=PROFILES["bottle"], floor=floor)
    run(mon, t, q, qd, tau)
    assert any(1.5 <= e.t <= 2.1 for e in mon.events)


def test_payload_in_the_model_quiets_the_carry(model, floor):
    """Carried but unweighed, the bottle's swinging weight is in the fast band."""
    t, q, qd, tau = pick_and_carry(model)
    carrying = t > 3.5
    unweighed = {
        1.2: lambda m: m.begin_load_change(),
        2.5: lambda m: setattr(m, "_tare", None),  # stop projecting, never load it
    }
    peaks = {}
    for weigh, hooks in ((True, GRASP), (False, unweighed)):
        mon = ContactMonitor(model, profile=Profile("off", np.inf), floor=floor)
        fast = []
        run(
            mon,
            t,
            q,
            qd,
            tau,
            hooks,
            record=lambda m, k: fast.append(m.fast / m.floor.at(np.abs(qd[k]).max())),
        )
        peaks[weigh] = np.abs(np.array(fast)[carrying]).max()
    # On this 0.5 kg bottle the 0.2 s slow band already absorbs most of the swing;
    # loading the payload takes off the rest (about 1.7x here, more for heavier loads).
    assert peaks[True] < PROFILES["bottle"].scale
    assert peaks[False] > 1.5 * peaks[True]


def test_bump_during_the_grasp_still_fires(model, floor):
    """The load-change projection removes what a held load can do, not everything."""
    t, q, qd, tau = pick_and_carry(
        model, collision=(1.8, 1.95, [25.0, 0.0, 0.0]), duration=3.0
    )
    mon = ContactMonitor(model, profile=PROFILES["bottle"], floor=floor)
    run(mon, t, q, qd, tau, GRASP)
    assert any(1.8 <= e.t <= 1.9 and e.during_load_change for e in mon.events)


def still_run(model, ext_fn, duration=6.0):
    t = np.arange(0.0, duration, 1.0 / RATE)
    q = np.tile(Q0, (len(t), 1))
    z = np.zeros_like(q)
    ext = np.array([ext_fn(tk) for tk in t])
    return t, q, z, measured_tau(model, q, z, z, np.zeros(len(t)), ext)


def test_leaning_does_not_fire_a_tap_does(model, floor):
    def ext(tk):
        e = np.zeros(7)
        e[3] = 6.0 * np.clip((tk - 1.0) / 2.0, 0.0, 1.0)  # lean in over 2 s, then hold
        if 4.5 <= tk < 4.65:
            e[3] += 3.0  # a sharp tap on top of the lean
        return e

    t, q, qd, tau = still_run(model, ext)
    mon = ContactMonitor(model, floor=floor)
    run(mon, t, q, qd, tau)
    assert [round(e.t, 2) for e in mon.events if e.t < 4.5] == []
    assert (
        len(mon.events) == 1
        and 4.5 <= mon.events[0].t <= 4.55
        and mon.events[0].joint == 3
    )


def test_pinned_after_a_collision_is_one_event(model, floor):
    """Hit and then held there: one event, closed after max_event_s, not a stream."""
    t, q, qd, tau = still_run(
        model, lambda tk: np.r_[0, 0, 0, 5.0 * (tk >= 2.0), 0, 0, 0]
    )
    mon = ContactMonitor(model, floor=floor)
    run(mon, t, q, qd, tau)
    assert len(mon.events) == 1
    ev = mon.events[0]
    assert (
        ev.t_end is not None and ev.t_end - ev.t <= PROFILES["empty"].max_event_s + 0.02
    )


def test_fork_catches_a_bump_the_empty_profile_ignores(model, floor):
    def thr(p):
        return max(p.scale * floor.still[3], p.min_nm)

    # The fast band shows about 65% of a 0.2 s step; 1.1 N.m lands between the two.
    tap = 1.1
    assert thr(PROFILES["fork"]) < 0.65 * tap < thr(PROFILES["empty"])
    t, q, qd, tau = still_run(
        model, lambda tk: np.r_[0, 0, 0, tap * (3.0 <= tk < 3.2), 0, 0, 0]
    )
    counts = {}
    for name in ("empty", "fork"):
        mon = ContactMonitor(model, profile=PROFILES[name], floor=floor)
        run(mon, t, q, qd, tau)
        counts[name] = len(mon.events)
    assert counts == {"empty": 0, "fork": 1}


def test_floor_round_trips_through_json(floor):
    back = Floor.from_json(floor.to_json())
    assert np.allclose(back.still, floor.still) and np.allclose(
        back.moving, floor.moving
    )
