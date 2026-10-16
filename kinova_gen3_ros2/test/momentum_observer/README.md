# Momentum observer: a passive look at external torque

An experiment, not a feature. Nothing here commands the arm. It replays a
recording of `/joint_states` through a generalized-momentum observer and plots
the result beside the external wrench core already publishes, so the two can be
compared on the same move.

## What it is for

Core estimates external torque as `g(q) - tau` (`supervisor.cpp`, the wrench
on `/ee_state`). At rest that is exact. While the arm moves, its own inertia
and Coriolis torque land in the same number, so a brisk trajectory looks like a
push and a real push mid-move is buried. The observer uses the full dynamics,
without differentiating velocity, and keeps working while the arm moves. The
derivation and sign convention are in `observer.py`.

Model error (`kinova-gen3-driver#5`) is a bias in both signals. The observer does
not remove it.

## Run it

Everything needs Python with `numpy`, `pinocchio` and `matplotlib`. The
description package's tests already use Pinocchio, so the container has it.

1. Render the model the node is running on. The observer must use the same
   masses as the driver.

   ```sh
   ros2 run kinova_gen3_description kinova_gen3_render.py --mount <robot>/mount.yaml --out /tmp/arm.urdf
   ```

1. Record. Start the recorder, run any move you already run (for example
   `send_joint_tour.py`), and have someone tap the arm once or twice mid-move.
   Press Enter in the recorder's terminal at each tap; that stamps a marker.

   ```sh
   python3 record_joint_states.py --out run.csv
   ```

   The recorder only subscribes. The move is whatever you were going to do
   anyway.

1. Analyze.

   ```sh
   python3 analyze.py --csv run.csv --urdf /tmp/arm.urdf --out run.png
   ```

   `run.png` has three stacked traces: joint speed, core's `g(q) - tau`, and
   the observer. `run.md` is a table of motion RMS (the signal while the arm
   moves with no tap), tap peak, and their ratio, for both estimators.

For a page instead of a PNG:

```sh
python3 report.py --csv run.csv --urdf /tmp/arm.urdf --out run.html --notes notes.md
```

`run.html` is self-contained: tiles, the three traces with a crosshair tooltip,
the summary and per-tap tables, and a notes column from `--notes`. The style
files under `web/` are vendored from alex-style-guide.

No arm handy? `synthetic.py` writes a `run.csv` from the model with two injected
elbow pushes, and `analyze.py` runs on it the same way. `test_observer.py` checks
the observer recovers an injected push in motion; set `KINOVA_URDF` to a URDF
path and run `pytest`.

## What to expect

On the synthetic sweep (0.9 rad/s peak, 0.15 N·m sensor noise, 50 Hz), core's
residual on the elbow swings about 0.9 N·m RMS from the arm's own motion and the
4 N·m push stands out by a factor of 6. The observer's motion RMS is 0.06 N·m and
the same push stands out by a factor of 67. On the arm the model error adds a
pose-dependent bias to both; the ratio is the number to compare.

## Limits

- `/joint_states` is decimated to about 50 Hz. The observer gain defaults to
  10 rad/s, which is as fast as that rate allows. A hand tap shows; a hard impact
  is smeared. The 1 kHz version belongs in core's RT loop, next to `FeedbackTap`.
- The sign of the measured torque depends on the core version. Core v1.2.0,
  which this branch pins, forwards KORTEX's raw torque, about -g(q) at rest; core
  `dev` flips it at the transport. `analyze.py` detects the sign from the still
  samples and prints which it found. `--tau-sign` overrides it.
- Joint 1 carries no gravity torque on a level mount, so its residual is
  mostly sensor offset.
