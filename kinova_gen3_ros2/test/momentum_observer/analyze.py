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
SETTLE_S = 0.5  # observer start-up transient, excluded from every statistic
# Window around a marker that counts as the tap. Measured on abra: the poke lands
# 0.15 to 0.30 s BEFORE the button press (reaction time), so the window leads.
TAP_BEFORE_S, TAP_AFTER_S = 0.6, 0.6


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    q = np.array([[float(r[f"q{i}"]) for i in range(1, 8)] for r in rows])
    qd = np.array([[float(r[f"qd{i}"]) for i in range(1, 8)] for r in rows])
    tau = np.array([[float(r[f"tau{i}"]) for i in range(1, 8)] for r in rows])
    marker = np.array([int(float(r.get("marker", 0) or 0)) for r in rows])
    phase = t[marker == 2]
    return t, q, qd, tau, t[marker == 1], (float(phase[0]) if len(phase) else None)


def detect_tau_sign(model, q, qd, tau):
    """+1 if tau is in the command convention (~ +g at rest), -1 if reaction-signed.

    Core v1.2.0 forwards KORTEX's raw torque, which reads ~ -g(q) at rest; core dev
    (the EE-wrench change, #74) flips it at the transport. The two differ only in
    sign, and a still sample tells them apart: the dot product of tau and g(q).
    """
    still = np.abs(qd).max(axis=1) < 0.02
    if still.sum() < 10:
        still = np.ones(len(q), bool)
    dots = [float(tau[k] @ model.gravity(q[k])) for k in np.flatnonzero(still)]
    return 1.0 if np.median(dots) >= 0 else -1.0


def rest_offset(t, qd, sig):
    """Per-joint median of sig over the first still stretch: the model error at the start pose.

    Both estimators carry it (same g(q), same URDF), so it is subtracted from both
    before any statistic. It is reported, not hidden: it IS the mass-model gap.
    """
    still = (np.abs(qd).max(axis=1) < 0.01) & (t > SETTLE_S)
    first_move = np.flatnonzero(~still & (t > SETTLE_S))
    if len(first_move):
        still &= t < t[first_move[0]]
    if still.sum() < 10:
        still = t > SETTLE_S
    return np.median(sig[still], axis=0)


def windows(t, qd, taps, before=TAP_BEFORE_S, after=TAP_AFTER_S, phase=None):
    """moving: the arm is in motion. tap: inside a tap window.

    With a phase boundary and no tap markers, everything after the boundary is the
    tap window (the operator tapped somewhere in there). With a boundary AND tap
    markers, the no-tap statistics come from the baseline loop only.
    """
    moving = (np.abs(qd).max(axis=1) > MOVING_RAD_S) & (t > SETTLE_S)
    tap = np.zeros(len(t), bool)
    for m in taps:
        tap |= (t >= m - before) & (t <= m + after)
    if phase is not None:
        if not len(taps):
            tap |= t >= phase
        else:
            moving &= t < phase
    return moving, tap


def summarize(name, sig, moving, tap, joints):
    """RMS and peak while moving with no tap, peak inside the tap windows, and the peak ratio.

    A detector thresholds on peaks, so the ratio that matters is tap peak over the
    largest excursion the arm's own motion produced with nobody touching it.
    """
    quiet = moving & ~tap
    out = {}
    for j in joints:
        rms = (
            float(np.sqrt(np.mean(sig[quiet, j] ** 2))) if quiet.any() else float("nan")
        )
        qpeak = float(np.abs(sig[quiet, j]).max()) if quiet.any() else float("nan")
        peak = float(np.abs(sig[tap, j]).max()) if tap.any() else float("nan")
        out[j] = (rms, qpeak, peak, peak / qpeak if qpeak > 0 else float("nan"))
    return out


def write_summary(
    path,
    csv_path,
    gain,
    taps,
    qs_stats,
    ob_stats,
    joints,
    rate,
    before,
    after,
    qs_off,
    ob_off,
):
    lines = [
        f"# {os.path.basename(csv_path)}",
        "",
        f"Observer gain {gain:g} rad/s, sample rate {rate:.0f} Hz, {len(taps)} tap marker(s).",
        "",
        "Motion RMS is the signal while the arm moves with no tap in progress: the part",
        "that is the arm's own inertia, not a push. Tap peak is the largest excursion",
        f"within {before:g} s before to {after:g} s after a marker. Ratio is tap peak",
        "over motion peak: how far a push stands out above the arm's own motion.",
        "",
        "Rest offset is each signal's value with the arm still at the start pose, the",
        "mass-model error there; it is subtracted from both before the other columns,",
        "since both carry the same model. Motion peak is the largest excursion with",
        "nobody touching the arm: the false-alarm floor a detector has to sit above.",
        "",
        "| Joint | Signal | Rest offset, N.m | Motion RMS, N.m | Motion peak, N.m | Tap peak, N.m | Ratio |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for j in joints:
        for label, st, off in (
            ("core g(q) - tau", qs_stats, qs_off),
            ("observer", ob_stats, ob_off),
        ):
            rms, qpeak, peak, ratio = st[j]
            lines.append(
                f"| {j + 1} | {label} | {off[j]:+.2f} | {rms:.2f} | {qpeak:.2f} "
                f"| {peak:.2f} | {ratio:.1f} |"
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
        "Quasi-static residual, rest offset removed; carries the arm's inertia",
    )
    ps.title(
        a2,
        "Momentum observer",
        "Same q, q̇, τ, rest offset removed; inertia and Coriolis accounted for",
    )
    ps.page_title(fig, "External torque during a move")
    ps.save(fig, path)


def baseline_map(t, q, qd, sig, phase, radius=0.03):
    """Subtract the baseline loop's residual at the same pose and direction.

    The model error is a function of pose, not time, and the tour repeats the same
    path, so for every sample after the phase boundary take the baseline sample
    nearest in joint space (same sign of velocity on the moving joints) and subtract
    its residual. Both estimators get the identical correction. Samples with no
    baseline neighbour within `radius` rad (RMS over joints) are left alone.
    """
    base = (t < phase) & (t > SETTLE_S)  # skip the observer's start-up transient
    if base.sum() < 10:
        return sig.copy(), np.zeros(len(t), bool)
    feat = np.hstack(
        [q, 0.05 * np.sign(np.round(qd, 2))]
    )  # direction as a small extra feature
    fb, sb = feat[base], sig[base]
    out = sig.copy()
    matched = np.zeros(len(t), bool)
    for k in np.flatnonzero(~base):
        d = np.sqrt(((fb - feat[k]) ** 2).mean(axis=1))
        j = int(np.argmin(d))
        if d[j] <= radius:
            out[k] = sig[k] - sb[j]
            matched[k] = True
    return out, matched


def compute(
    csv_path, urdf, gain=20.0, tau_sign=0.0, before=TAP_BEFORE_S, after=TAP_AFTER_S
):
    """Everything the plot, the table and the HTML report share. One dict, no plotting."""
    model = ArmModel(urdf)
    t, q, qd, tau, taps, phase = load_csv(csv_path)
    sign = tau_sign or detect_tau_sign(model, q, qd, tau)
    tau = tau * sign
    rate = 1.0 / np.median(np.diff(t))
    qs = np.array([quasi_static_residual(model, q[k], tau[k]) for k in range(len(t))])
    ob = run_observer(model, t, q, qd, tau, gain=gain)
    qs_off, ob_off = rest_offset(t, qd, qs), rest_offset(t, qd, ob)
    qs, ob = qs - qs_off, ob - ob_off
    moving, tap = windows(t, qd, taps, before, after, phase)
    qs_bs = ob_bs = matched = None
    if phase is not None:
        qs_bs, matched = baseline_map(t, q, qd, qs, phase)
        ob_bs, _ = baseline_map(t, q, qd, ob, phase)
    return {
        "csv": csv_path,
        "urdf": urdf,
        "gain": gain,
        "sign": sign,
        "rate": rate,
        "t": t,
        "q": q,
        "qd": qd,
        "tau": tau,
        "taps": taps,
        "phase": phase,
        "qs": qs,
        "ob": ob,
        "qs_bs": qs_bs,
        "ob_bs": ob_bs,
        "matched": matched,
        "qs_off": qs_off,
        "ob_off": ob_off,
        "moving": moving,
        "tap": tap,
        "before": before,
        "after": after,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--urdf", required=True, help="the rendered URDF the node runs on")
    ap.add_argument(
        "--out", required=True, help="PNG path; the .md summary sits beside it"
    )
    ap.add_argument("--gain", type=float, default=20.0, help="observer cutoff, rad/s")
    ap.add_argument("--joints", default="2,4,6", help="1-based joints to plot, up to 6")
    ap.add_argument(
        "--tau-sign",
        type=float,
        default=0.0,
        choices=[0.0, 1.0, -1.0],
        help="torque sign convention; 0 (default) detects it from the still samples",
    )
    ap.add_argument("--light", action="store_true", help="light theme for slides")
    ap.add_argument(
        "--tap-before",
        type=float,
        default=TAP_BEFORE_S,
        help="tap window before a marker, s",
    )
    ap.add_argument(
        "--tap-after",
        type=float,
        default=TAP_AFTER_S,
        help="tap window after a marker, s",
    )
    args = ap.parse_args()

    joints = [int(s) - 1 for s in args.joints.split(",")]
    if len(joints) > 6 or any(j < 0 or j > 6 for j in joints):
        sys.exit("--joints: up to six values in 1..7")

    r = compute(
        args.csv, args.urdf, args.gain, args.tau_sign, args.tap_before, args.tap_after
    )
    print(
        f"torque sign {r['sign']:+.0f}: "
        + (
            "command convention (~ +g at rest)"
            if r["sign"] > 0
            else "reaction-signed (core v1.2.0 raw KORTEX)"
        )
    )
    stem = os.path.splitext(args.out)[0]
    qs_stats = summarize("core", r["qs"], r["moving"], r["tap"], joints)
    ob_stats = summarize("observer", r["ob"], r["moving"], r["tap"], joints)
    if r["ob_bs"] is not None:
        # tap-phase statistics only make sense after the boundary; the no-tap floor of
        # the corrected signal is the tap-loop motion with no tap in progress
        t, ph = r["t"], r["phase"]
        mv = (np.abs(r["qd"]).max(axis=1) > MOVING_RAD_S) & (t >= ph)
        bs = summarize("observer - baseline", r["ob_bs"], mv, r["tap"], joints)
        print("observer, baseline map subtracted (tap loops only):")
        for j in joints:
            _rms, qpeak, peak, ratio = bs[j]
            print(
                f"  J{j + 1}: motion peak {qpeak:.2f}  tap peak {peak:.2f}  ratio {ratio:.1f}"
            )
    write_summary(
        stem + ".md",
        args.csv,
        args.gain,
        r["taps"],
        qs_stats,
        ob_stats,
        joints,
        r["rate"],
        args.tap_before,
        args.tap_after,
        r["qs_off"],
        r["ob_off"],
    )
    plot(args.out, r["t"], r["qd"], r["qs"], r["ob"], r["taps"], joints, args.light)
    print(f"wrote {args.out} and {stem}.md")


if __name__ == "__main__":
    main()
