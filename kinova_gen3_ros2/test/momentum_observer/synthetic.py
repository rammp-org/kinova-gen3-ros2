#!/usr/bin/env python3
"""Write a synthetic run.csv so the pipeline runs end to end with no arm.

A smooth sweep on joints 2, 4 and 6 at 50 Hz, with two 0.3 s pushes injected on
the elbow (joint 4) and marked the way record_joint_states.py marks a real tap.
Sensor noise of 0.15 N.m RMS is added to the torque so the plot is honest about
what 50 Hz buys.

  python3 synthetic.py --urdf /tmp/arm.urdf --out synthetic.csv
"""

import argparse
import csv

import numpy as np

from observer import ArmModel

RATE = 50.0
PUSH_JOINT = 3  # 0-based: joint 4
PUSH_NM = 4.0
PUSHES = [(2.4, 2.7), (5.1, 5.4)]  # 0.3 s elbow pushes, one mid-swing, one near a turn
Q0 = np.array([0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57])


def sweep(model, duration=8.0, rate=RATE, noise_nm=0.0, seed=0):
    """Return t, q, qd, tau, ext, push_mask for a sweep with elbow pushes."""
    t = np.arange(0.0, duration, 1.0 / rate)
    amp = np.zeros(7)
    amp[[1, 3, 5]] = [0.35, 0.45, 0.4]
    w = 2 * np.pi / 1.5  # 1.5 s period: ~0.9 rad/s peak, like a brisk joint tour
    q = Q0 + amp * (1 - np.cos(w * t))[:, None] / 2
    qd = amp * w * np.sin(w * t)[:, None] / 2
    qdd = amp * w * w * np.cos(w * t)[:, None] / 2
    ext = np.zeros_like(q)
    push = np.zeros(len(t), bool)
    for a, b in PUSHES:
        push |= (t >= a) & (t < b)
    ext[push, PUSH_JOINT] = PUSH_NM
    tau = np.array(
        [model.inverse_dynamics(q[k], qd[k], qdd[k]) - ext[k] for k in range(len(t))]
    )
    if noise_nm > 0:
        tau = tau + np.random.default_rng(seed).normal(0.0, noise_nm, tau.shape)
    return t, q, qd, tau, ext, push


def write_csv(path, t, q, qd, tau, push):
    starts = np.flatnonzero(np.diff(push.astype(int)) == 1) + 1
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["t"]
            + [f"q{i}" for i in range(1, 8)]
            + [f"qd{i}" for i in range(1, 8)]
            + [f"tau{i}" for i in range(1, 8)]
            + ["fx", "fy", "fz", "tx", "ty", "tz", "marker"]
        )
        for k in range(len(t)):
            w.writerow(
                [f"{t[k]:.4f}"]
                + [f"{v:.6f}" for v in q[k]]
                + [f"{v:.6f}" for v in qd[k]]
                + [f"{v:.4f}" for v in tau[k]]
                + ["nan"] * 6
                + [1 if k in starts else 0]
            )


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--noise", type=float, default=0.15, help="torque noise, N.m RMS")
    args = ap.parse_args()
    model = ArmModel(args.urdf)
    t, q, qd, tau, _, push = sweep(model, noise_nm=args.noise)
    write_csv(args.out, t, q, qd, tau, push)
    print(f"{args.out}: {len(t)} rows, {len(PUSHES)} markers")


if __name__ == "__main__":
    main()
