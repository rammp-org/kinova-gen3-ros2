# Momentum observer: a passive look at external torque

An experiment, not a feature (yet: see [Collision detection and payload
weighing](#collision-detection-and-payload-weighing)). Nothing here commands
the arm. It replays a
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

## Panel

Buttons instead of commands. On the arm computer:

```sh
source env.sh
python3 panel.py --port 8790
```

Then from your laptop `ssh -L 8790:localhost:8790 abra` and open
`http://localhost:8790/`. One button runs the whole test: record, a baseline
loop with nobody touching the arm, the loops you tap during, then the report,
which appears on the page. A large TAP button stamps the marker, so a phone on
the same tunnel or LAN (`--bind 0.0.0.0`) works as the tap clicker. The
step-by-step panel exposes record, tour, and analyze separately. `--mock` runs
the page with synthetic data and no ROS.

## 1 kHz from the driver

`/joint_states` is fed by the Supervisor's non-RT pump at about 100 Hz, so the
topic cannot go faster. Branch `feat/feedback-recorder` (v1.1.0 plus one
decorator in the RT loop) records every feedback frame to a CSV when two node
parameters are set:

```sh
ros2 param set /kinova_gen3_node feedback_record_path /runs/run5.csv
ros2 param set /kinova_gen3_node feedback_record true     # ... false to stop
```

`arm_recorder.sh start` stops sheppy's arm container and runs that image as
`momentum-arm` with the manifest's flags plus `runs/` mounted at `/runs`;
`arm_recorder.sh stop` hands the arm back with `sheppy restart arm`. Nothing in
rammp-deployments changes. The panel has the same two buttons, and while the
recorder container is up its Record and TAP buttons use the driver path: taps
go to `<run>.markers` keyed by wall time and the analyzer merges them.

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

## Collision detection and payload weighing

`contact.py` builds two features on the observer, and stops there: what to do
about a collision is a later decision. It is passive like everything else here.
The module docstring has the design; in short:

- **Impulses only.** The residual is split into a slow band (0.2 s average) and
  a fast band. Only the fast band fires, so leaning on the arm, hand-guiding,
  resting on a table and the model's pose-dependent bias never do. A contact
  that stays after a hit becomes the new baseline after `max_event_s` instead
  of firing again.
- **Payload weighing.** Tare with the arm still before the gripper closes,
  weigh with it still after the lift. The object goes into the model as a
  point mass, so its weight and inertia stop reading as external torque while
  it is carried. Between tare and weigh the detector ignores only what a held
  load can do (vertical force, horizontal moment at the gripper); a bump from
  the side still fires.
- **Profiles per held object.** Threshold per joint is `scale` times the
  calibrated floor, never below `min_nm`. `fork` is fine (1.2x, 0.4 N·m);
  `empty` and `bottle` are coarse (2x, 1 N·m). The numbers are starting points.

```python
mon = ContactMonitor(ArmModel(urdf), profile=PROFILES["empty"], floor=floor)
ev = mon.step(t, q, qd, tau)          # every sample; a CollisionEvent at onset
mon.begin_load_change()               # still, before the gripper closes
est = mon.end_load_change()           # still, after the lift: est.payload.mass
mon.set_profile(PROFILES["bottle"])
```

`test_contact.py` covers it on synthetic runs with a pose-dependent model error
like abra's: a 0.5 kg bottle weighed to within 0.05 kg through a 4 N·m shoulder
bias, carried at speed with no false alarm, and a 25 N side bump caught within
80 ms.

### On the arm

`replay_contact.py` scores a recording offline, with the same CSVs and tap
markers as above:

1. **Floor.** Run the full test in the panel (baseline loop, phase marker,
   tap loops). Replay calibrates the floor on the baseline loop and scores
   each profile on the taps:
   `python3 replay_contact.py --csv runs/run6.csv --urdf arm.urdf --save-floor floor.json`
1. **Pick.** Record a pick of a bottle by hand-commanded moves: still, close
   the gripper, lift, still, then the tour with taps on the bottle and the
   forearm. Note the two still times and replay with
   `--floor floor.json --grasp TARE,WEIGH`. It prints the weight, and every
   event that is not a tap is a false alarm.

## Limits

- The floor sets the finest threshold any profile can use without false
  alarms, and on the arm it is the mass model (about 3 N·m on the shoulder in
  motion). Moving slower lowers it; so would a corrected URDF from
  `gravity_id` (kinova-gen3-driver `feat/torque-offset-calibration`).
- `/joint_states` is decimated to about 50 Hz. The observer gain defaults to
  10 rad/s, which is as fast as that rate allows. A hand tap shows; a hard impact
  is smeared. The 1 kHz version belongs in core's RT loop, next to `FeedbackTap`.
- The sign of the measured torque depends on the core version. Core v1.2.0,
  which this branch pins, forwards KORTEX's raw torque, about -g(q) at rest; core
  `dev` flips it at the transport. `analyze.py` detects the sign from the still
  samples and prints which it found. `--tau-sign` overrides it.
- Joint 1 carries no gravity torque on a level mount, so its residual is
  mostly sensor offset.
