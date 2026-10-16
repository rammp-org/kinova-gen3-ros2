#!/usr/bin/env python3
"""Replay a recorded run through the momentum observer and plot it against core's wrench.

  python3 analyze.py --csv run.csv --urdf /tmp/arm.urdf --out run.png

Produces run.png (three stacked traces: joint speed, core's quasi-static residual,
the observer) and run.md, a table of how much of each signal is the arm's own
motion and how much is the tap.
"""

import argparse
import csv
import os
import sys

import numpy as np

from observer import ArmModel, quasi_static_residual, run_observer

MOVING_RAD_S = 0.05  # above this the arm counts as moving
TAP_BEFORE_S, TAP_AFTER_S = 0.2, 0.8  # window around a marker that counts as the tap


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    q = np.array([[float(r[f"q{i}"]) for i in range(1, 8)] for r in rows])
    qd = np.array([[float(r[f"qd{i}"]) for i in range(1, 8)] for r in rows])
    tau = np.array([[float(r[f"tau{i}"]) for i in range(1, 8)] for r in rows])
    marker = np.array([int(float(r.get("marker", 0) or 0)) for r in rows])
    return t, q, qd, tau, t[marker == 1]


def windows(t, qd, taps):
    moving = np.abs(qd).max(axis=1) > MOVING_RAD_S
    tap = np.zeros(len(t), bool)
    for m in taps:
        tap |= (t >= m - TAP_BEFORE_S) & (t <= m + TAP_AFTER_S)
    return moving, tap


def summarize(name, sig, moving, tap, joints):
    """RMS while moving with no tap, peak inside the tap windows, and their ratio."""
    quiet = moving & ~tap
    out = {}
    for j in joints:
        rms = (
            float(np.sqrt(np.mean(sig[quiet, j] ** 2))) if quiet.any() else float("nan")
        )
        peak = float(np.abs(sig[tap, j]).max()) if tap.any() else float("nan")
        out[j] = (rms, peak, peak / rms if rms > 0 else float("nan"))
    return out


def write_summary(path, csv_path, gain, taps, qs_stats, ob_stats, joints, rate):
    lines = [
        f"# {os.path.basename(csv_path)}",
        "",
        f"Observer gain {gain:g} rad/s, sample rate {rate:.0f} Hz, {len(taps)} tap marker(s).",
        "",
        "Motion RMS is the signal while the arm moves with no tap in progress: the part",
        "that is the arm's own inertia, not a push. Tap peak is the largest excursion",
        f"within {TAP_BEFORE_S:g} s before to {TAP_AFTER_S:g} s after a marker. Ratio is tap peak",
        "over motion RMS: how far a push stands out from the arm's own motion.",
        "",
        "| Joint | Signal | Motion RMS, N.m | Tap peak, N.m | Ratio |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for j in joints:
        for label, st in (("core g(q) - tau", qs_stats), ("observer", ob_stats)):
            rms, peak, ratio = st[j]
            lines.append(
                f"| {j + 1} | {label} | {rms:.2f} | {peak:.2f} | {ratio:.1f} |"
            )
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


def plot(path, t, qd, qs, ob, taps, joints, light):
    import plot_style as ps

    pal = ps.use("light" if light else "dark")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    fig.subplots_adjust(hspace=0.42, left=0.08, right=0.9, top=0.89, bottom=0.07)
    a0, a1, a2 = axes

    for k, j in enumerate(joints):
        a0.plot(t, qd[:, j], color=pal["series"][k], label=f"J{j + 1}")
        a1.plot(t, qs[:, j], color=pal["series"][k], label=f"J{j + 1}")
        a2.plot(t, ob[:, j], color=pal["series"][k], label=f"J{j + 1}")
    lim = 1.1 * max(np.abs(qs[:, joints]).max(), np.abs(ob[:, joints]).max(), 0.5)
    a1.set_ylim(-lim, lim)
    a2.set_ylim(-lim, lim)
    for a in axes:
        ps.tap_lines(a, taps)
        ps.label_lines(a)
    a0.set_ylabel("Joint speed, rad/s")
    a1.set_ylabel("External torque, N·m")
    a2.set_ylabel("External torque, N·m")
    a2.set_xlabel("Time, s")
    ps.title(a0, "Joint speed", "Context: when the arm is moving under its own command")
    ps.title(
        a1,
        "Core: g(q) − τ",
        "Quasi-static residual as published on /ee_state; carries the arm's inertia",
    )
    ps.title(
        a2, "Momentum observer", "Same q, q̇, τ; inertia and Coriolis accounted for"
    )
    ps.page_title(fig, "External torque during a move")
    ps.save(fig, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--urdf", required=True, help="the rendered URDF the node runs on")
    ap.add_argument(
        "--out", required=True, help="PNG path; the .md summary sits beside it"
    )
    ap.add_argument("--gain", type=float, default=10.0, help="observer cutoff, rad/s")
    ap.add_argument("--joints", default="2,4,6", help="1-based joints to plot, up to 6")
    ap.add_argument(
        "--tau-sign",
        type=float,
        default=1.0,
        choices=[1.0, -1.0],
        help="flip once if the arm's torque convention proves inverted",
    )
    ap.add_argument("--light", action="store_true", help="light theme for slides")
    args = ap.parse_args()

    joints = [int(s) - 1 for s in args.joints.split(",")]
    if len(joints) > 6 or any(j < 0 or j > 6 for j in joints):
        sys.exit("--joints: up to six values in 1..7")

    model = ArmModel(args.urdf)
    t, q, qd, tau, taps = load_csv(args.csv)
    tau = tau * args.tau_sign
    rate = 1.0 / np.median(np.diff(t))
    qs = np.array([quasi_static_residual(model, q[k], tau[k]) for k in range(len(t))])
    ob = run_observer(model, t, q, qd, tau, gain=args.gain)
    moving, tap = windows(t, qd, taps)

    stem = os.path.splitext(args.out)[0]
    qs_stats = summarize("core", qs, moving, tap, joints)
    ob_stats = summarize("observer", ob, moving, tap, joints)
    write_summary(
        stem + ".md", args.csv, args.gain, taps, qs_stats, ob_stats, joints, rate
    )
    plot(args.out, t, qd, qs, ob, taps, joints, args.light)
    print(f"wrote {args.out} and {stem}.md")


if __name__ == "__main__":
    main()
