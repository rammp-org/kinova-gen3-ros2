#!/usr/bin/env python3
"""Replay a recorded run through the collision detector and score it against the tap markers.

Offline. Reads a CSV written by record_joint_states.py or the driver recorder;
nothing here talks to the arm.

  # calibrate the floor from the hands-off baseline loop (before the phase marker),
  # save it, and score every profile on the tap loops after it
  python3 replay_contact.py --csv runs/run6.csv --urdf arm.urdf --save-floor floor.json

  # reuse a saved floor on another run
  python3 replay_contact.py --csv runs/run7.csv --urdf arm.urdf --floor floor.json

  # a pick: arm still at 12.0 s before the gripper closed, still again at 16.5 s
  # after the lift; weighs the object and carries it for the rest of the run
  python3 replay_contact.py --csv runs/pick1.csv --urdf arm.urdf --floor floor.json \
      --grasp 12.0,16.5

A tap counts as caught if an event opens within the tap window around its
marker (the same window analyze.py uses: the poke lands before the button). Any
other event is a false alarm.
"""

import argparse

import numpy as np

from analyze import TAP_AFTER_S, TAP_BEFORE_S, detect_tau_sign, load_csv
from contact import PROFILES, ContactMonitor, Floor, calibrate_floor
from observer import ArmModel


def score(events, taps, before=TAP_BEFORE_S, after=TAP_AFTER_S, t_from=-np.inf):
    """(caught taps, missed taps, false alarms) for events at or after t_from."""
    evs = [e for e in events if e.t >= t_from]
    caught = [m for m in taps if any(m - before <= e.t <= m + after for e in evs)]
    false = [e for e in evs if not any(m - before <= e.t <= m + after for m in taps)]
    return caught, [m for m in taps if m not in caught], false


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--urdf", required=True, help="the rendered URDF the node runs on")
    ap.add_argument("--floor", help="floor JSON from an earlier --save-floor")
    ap.add_argument("--save-floor", help="write the floor calibrated from this run")
    ap.add_argument(
        "--profile", default="all", choices=["all", *PROFILES], help="profile to score"
    )
    ap.add_argument(
        "--grasp",
        action="append",
        default=[],
        metavar="TARE,WEIGH",
        help="times, s, when the arm was still before and after a grasp or release; "
        "repeatable",
    )
    ap.add_argument("--gain", type=float, default=20.0, help="observer cutoff, rad/s")
    ap.add_argument(
        "--tau-sign",
        type=float,
        default=0.0,
        choices=[0.0, 1.0, -1.0],
        help="torque sign convention; 0 (default) detects it from the still samples",
    )
    args = ap.parse_args()

    model = ArmModel(args.urdf)
    t, q, qd, tau, taps, phase = load_csv(args.csv)
    tau = tau * (args.tau_sign or detect_tau_sign(model, q, qd, tau))

    if args.floor:
        with open(args.floor) as f:
            floor = Floor.from_json(f.read())
        t_from = -np.inf
        print(f"floor from {args.floor}")
    elif phase is not None:
        base = t < phase
        floor = calibrate_floor(
            model, t[base], q[base], qd[base], tau[base], gain=args.gain
        )
        t_from = phase
        print(f"floor calibrated on the baseline loop, t < {phase:.1f} s")
    else:
        floor = Floor.uncalibrated()
        t_from = -np.inf
        print("no --floor and no phase marker: UNCALIBRATED placeholder floor")
    print("  still  " + " ".join(f"{v:5.2f}" for v in floor.still) + "  N.m")
    print("  moving " + " ".join(f"{v:5.2f}" for v in floor.moving) + "  N.m")
    if args.save_floor:
        with open(args.save_floor, "w") as f:
            f.write(floor.to_json() + "\n")
        print(f"wrote {args.save_floor}")

    grasps = [tuple(float(v) for v in g.split(",")) for g in args.grasp]
    names = list(PROFILES) if args.profile == "all" else [args.profile]
    print(f"\n{len(taps)} tap marker(s) after t = {max(t_from, t[0]):.1f} s")
    print("| Profile | Caught | Missed | False alarms | Events (t s, joint, score) |")
    print("| --- | ---: | ---: | ---: | --- |")
    for name in names:
        mon = ContactMonitor(model, profile=PROFILES[name], floor=floor, gain=args.gain)
        hooks = sorted(
            [(a, "tare") for a, _ in grasps] + [(b, "weigh") for _, b in grasps]
        )
        weighed = []
        for k in range(len(t)):
            while hooks and t[k] >= hooks[0][0]:
                at, what = hooks.pop(0)
                if what == "tare":
                    if not mon.begin_load_change():
                        print(f"  {name}: arm not still at tare t = {at:.1f} s")
                else:
                    weighed.append((at, mon.end_load_change()))
            mon.step(t[k], q[k], qd[k], tau[k])
        caught, missed, false = score(mon.events, taps, t_from=t_from)
        evs = ", ".join(
            f"{e.t:.2f} J{e.joint + 1} {e.score:.1f}"
            for e in mon.events
            if e.t >= t_from
        )
        print(f"| {name} | {len(caught)} | {len(missed)} | {len(false)} | {evs} |")
        if name == names[0]:
            for at, est in weighed:
                c = ", ".join(f"{v * 1e3:.0f}" for v in est.payload.com)
                print(
                    f"  weighed at {at:.1f} s: {est.payload.mass:.3f} kg "
                    f"(change {est.delta_kg:+.3f}) at ({c}) mm, fit {est.fit_rms:.2f} N.m"
                    + ("" if est.ok else f"  NOT LOADED: {est.reason}")
                )


if __name__ == "__main__":
    main()
