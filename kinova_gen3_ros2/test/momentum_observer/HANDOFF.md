# Momentum observer: handoff

Written 2026-10-29 after the sessions of 2026-10-16 on abra. Everything here is
on two branches of `rammp-org/kinova-gen3-ros2`:

| Branch | What |
| --- | --- |
| `feat/momentum-observer-demo` | this folder: observer, recorders, analyzer, HTML report, browser panel, tour, container swap script |
| `feat/feedback-recorder` | v1.1.0 (the image abra runs) plus a 1 kHz CSV recorder inside the node, controlled by two parameters |

`rammp-deployments` was not changed. Abra's arm runs sheppy's normal container.

## Why this exists

Issue kinova-gen3-driver#40: gravity-comp hold droops and drifts. The driver's
external-torque estimate (`g(q) - tau`, the wrench on `/ee_state`) is exact at
rest and carries the arm's own inertia in motion. A generalized-momentum
observer (De Luca) uses the full dynamics and stays valid in motion. The demo
replays a normal move through both and compares them. `observer.py` has the
derivation and sign convention.  Even though droops and drifts has largely been fixed, we would like this capability

## What we measured on abra

Arm: Gen3 7-DoF, 2F-85, level mount, image `kinova-gen3-ros2:1.1.0` (core
v1.2.0). Torque from that core is reaction-signed; `analyze.py` detects the
sign from still samples.

1. **The mass model is the biggest error.** Arm still at home: measured shoulder
   torque 10.7 N·m, model 15.1. That 4.4 N·m offset, and its 1 to 3 N·m change
   with pose, is in both estimators identically. Issue kinova-gen3-driver#5.
2. **The observer removes the inertial part.** Core's residual spikes to 4.5 N·m
   on the shoulder at every leg start and stop with nobody touching the arm;
   the observer's floor is about 3 N·m, and its trace is smooth where core's is
   a thicket. A baseline map (the baseline loop's residual at the same pose,
   subtracted from the tap loops) takes the observer's slow pose error on J2
   from 1.0 to 0.08 N·m RMS.
3. **Finger pokes are below the floor.** Runs 2 to 4 with per-tap markers: pokes
   measured 1 to 4 N·m on the shoulder, about the same as the motion floor for
   both estimators. The poke lands 0.15 to 0.30 s before the button press.
4. **1 kHz did not change the floor.** From the driver at 1000 Hz, zero dropped
   frames: sensor noise with the arm still is 0.02 N·m RMS. In motion, 75% of
   the shoulder torque content sits at 5 to 15 Hz, inside what the 100 Hz topic
   already shows. The observer floor on the same data decimated to 100 Hz is the
   same number. The floor is unmodeled dynamics, not sample rate.

So: the observer works as advertised; the remaining floor is the model. A
sustained push of 5 N·m or more will show clearly. A finger tap will not.

## Files

| File | Role |
| --- | --- |
| `observer.py` | `ArmModel` (URDF reduced to the 7 arm joints, core's q packing), `MomentumObserver`, `quasi_static_residual` |
| `record_joint_states.py` | `/joint_states` to CSV at ~100 Hz; Enter or `--marker-file` stamps a tap; marker 2 is the baseline/tap boundary |
| `analyze.py` | `compute()` shared by everything: sign detection, rest offset, baseline map, tap windows; writes PNG and markdown |
| `report.py` | self-contained HTML page per run (alex-style, vendored in `web/`) |
| `panel.py`, `panel.html` | browser control panel: one-button test, TAP (spacebar), step-by-step, embedded report, arm swap buttons |
| `tour.py` | four joint-space waypoints around home through `GoToJointConfig`, speed scale 0.6 by default |
| `arm_recorder.sh` | stop sheppy's arm container, run the recorder image as `momentum-arm` with the manifest's flags; `stop` hands it back |
| `synthetic.py`, `test_observer.py` | synthetic sweep with injected pushes; 5 pytest cases, set `KINOVA_URDF` |
| `contact.py`, `test_contact.py` | impulse detector, payload weighing, per-object profiles; 11 pytest cases (branch `feat/collision-payload`) |
| `replay_contact.py` | score a recording against its tap markers per profile; calibrate and save the floor; `--grasp` replays a weigh |
| `env.sh` | abra host env: ROS, interfaces workspace, Cyclone on loopback |

## State on abra

- `~/momentum_observer/`: a copy of this folder plus `arm.urdf` (pulled from the
  1.1.0 image), `runs/` (run1 to run5 at ~100 Hz, `static1k` and `base1k` at
  1 kHz, each with `.csv` and `.html`).
- Panel running under nohup on port 8790 (8766 belongs to another service).
- Image `kinova-gen3-ros2:feedback-recorder`, built from `~/momentum_observer/src/kinova-gen3-ros2` (a separate clone; `~/kinova-gen3-ros2` is someone else's checkout, untouched).
- `~/rammp-deployments` has uncommitted edits that were there before us.

## Running it

```sh
ssh -L 8790:localhost:8790 abra
```

Open `http://localhost:8790/`. If the panel is down:

```sh
ssh abra
source ~/momentum_observer/env.sh
cd ~/momentum_observer && nohup python3 panel.py --port 8790 >panel.log 2>&1 &
```

Stop it with `pkill -9 -f "[p]anel.py --port 8790"` from a shell whose own
command line does not contain `panel.py --port` (an SSH one-liner that both
kills and starts will kill itself; use two calls).

**Run full test**: records, runs one baseline loop (hands off), stamps the
phase marker, runs the tap loops, stops, builds the report. Press space at
each push. Both tour buttons ask for confirmation; the arm moves.

**1 kHz**: press "Arm: recorder image" (stops sheppy's arm, starts
`momentum-arm`; the arm holds through the swap). Record and TAP then go through
the node's parameters and a `<run>.markers` sidecar. "Arm: sheppy" hands it
back with `sheppy restart arm`. Leave it on sheppy when done.

From a shell instead:

```sh
python3 tour.py --go --loops 2 --speed 0.6
python3 analyze.py --csv runs/run6.csv --urdf arm.urdf --out runs/run6.png
python3 report.py  --csv runs/run6.csv --urdf arm.urdf --out runs/run6.html --notes notes.md
```

## What to try next, in order

1. **Sustained pushes.** Lean on the forearm for half a second instead of
   poking. Compare tap peak to the motion floor in the report's summary table.
2. **Static mass fit** (closes #5 and lifts every number here). Ten still poses
   with the wrist pointed different ways, read-only; fit a wrist payload (mass,
   mass times CoM) and seven sensor offsets by least squares. `benchmark_grav_comp --dry-run`
   in the core driver prints the residuals, but note its sign is inverted for
   core v1.2.0's raw torque.
3. **Observer in core's RT loop** if a detector is wanted. The 1 kHz data says
   the rate itself buys nothing for the floor, so this is about latency and
   abort wiring, not noise.
4. **Admittance for hand-guiding** (position mode, reference moved by the
   torque-sensor residual) sidesteps the mass model as the load carrier. See
   the discussion in the session notes; not started.

## Known rough edges

- `test_observer.py` under abra's system pytest hits a plugin hook error;
  run it locally or in a venv.
- The HTML inlines the vendored JS with `</` escaped; `web/ax-charts.js` has
  one local extension (`marks`) over the style guide's copy.
- `panel.py` uses `sheppy status` polling every 5 s for the arm tile; with the
  recorder container up it reports "recorder image" instead.
