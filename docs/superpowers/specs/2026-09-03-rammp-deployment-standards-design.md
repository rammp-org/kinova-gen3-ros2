# Adopting the RAMMP Deployment Standards — Design

Bring `kinova-gen3-ros2` onto the RAMMP module standard: pre-commit, CI with
auto-builds, a declared contract, and the interface page the docs site pulls.

`rammp-module-template` offers a minimal path for existing repos — copy three
files, keep your layout. **We are deliberately not taking it.** This repo is
becoming a released, documented, deployed module; adopting the structure now
costs one PR, and diverging from it costs a translation step forever.

Template: `rammp-org/rammp-module-template`. Launcher: `rammp-org/sheppy`.

## What we adopt

| From the template | Change here |
|---|---|
| `.pre-commit-config.yaml` | verbatim |
| `.clang-format` | verbatim |
| `.github/workflows/lint.yml` | verbatim |
| `.github/workflows/build.yml` | one deviation, below |
| `Makefile` targets `run` / `check` / `smoke` / `lint` | added alongside ours |
| `scripts/smoke.sh`, `scripts/validate_fragment.py`, `tests/` | verbatim |
| `rammp-alternative.yaml` | written for this module |
| `docs/interface.md` | written for this module |
| branch model `main` / `dev` / `feature/<issue>-<desc>` | adopted |

`Dockerfile` moves from `docker/Dockerfile` to the repo root so the standard's
`docker build .` works unmodified. `docker/requirements.txt` and
`docker/vendor/` stay put; the Dockerfile already refers to them by path.

The existing Makefile targets — `sim`, `real`, `e2e`, `build-real`,
`stage-kortex` — are KEPT. They encode the RT run flags and the KORTEX staging
step, which the standard has no place for and which are the difference between
a working arm and a confusing failure.

## The contract

```yaml
id: kinova_gen3_ros2
tier: experimental
kind: docker
ros_node_name: kinova_gen3_node
container:
  image: ghcr.io/rammp-org/kinova-gen3-ros2:dev
  network_mode: host
  ipc: host
  cap_add: [SYS_NICE]
  ulimits:
    rtprio: 99
    memlock: -1
  command:
    - /ros2_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node
    - --sim
    - --urdf
    - /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf
params:
  arbitration_mode: disabled
  estop_clear_max_age_s: 1.0
  expect_gripper: true
publishes:
  [/joint_states, /ee_state, /control_status, /stream_status,
   /gripper_state, /diagnostics]
subscribes:
  [/estop, /setpoint/joint_position, /setpoint/joint_velocity,
   /setpoint/joint_torque, /setpoint/pose, /setpoint/twist,
   /setpoint/wrench, /setpoint/gripper]
```

`ipc: host` is required for DDS shared-memory transport. `cap_add: SYS_NICE`
permits `sched_setscheduler`; the `ulimits` are what let it succeed —
capabilities and rlimits are separate mechanisms, and `privileged` does NOT
raise `RLIMIT_RTPRIO` (measured: privileged leaves `rtprio` at 0).

`ulimits` support is pending in sheppy — `rammp-org/sheppy#10`. This spec
assumes it lands. Until it does, a fragment declaring it is silently dropped
and the control loop runs best-effort with no error.

### `command` is mandatory here, not optional

`sheppy/launch/docker/__init__.py` builds the container's argv from the
fragment's `command:` and then, when `params` are present, **appends**
`--ros-args --params-file /sheppy/params.yaml`. The final invocation is
`docker run [flags] IMAGE [command]`.

If the fragment declares `params` but no `command`, that argv is
`["--ros-args", "--params-file", "/sheppy/params.yaml"]` alone — which
REPLACES the image's `CMD`. Our `ENTRYPOINT` then runs `exec "$@"` on
`--ros-args` and the container dies at startup.

So the fragment spells out the full command. This is not redundancy with the
Dockerfile's `CMD`: `CMD` serves `docker run` and `make smoke`, the fragment's
`command` serves sheppy, and the moment `params` exist the two stop being
interchangeable.

`ros_node_name: kinova_gen3_node` is declared for the same reason. Without it,
`write_params_file` keys the file under the `/**` wildcard, which works but
applies our parameters to every node in the process. Naming the node is exact.
It also matters that `arbitration_mode` is READ-ONLY at launch — it can only
arrive through a params file at startup, which is precisely this path.

### Two things the fragment cannot say

**Actions and services are not expressible.** The schema has `publishes` and
`subscribes` only. This module's PRIMARY interface is four action servers
(`execute_joint_trajectory`, `go_to_ee_pose`, `go_to_joint_config`,
`go_to_preset`) and six services (`/acquire_control`, `/release_control`,
`/revoke_control`, `/open_stream`, `/close_stream`, `/list_controllers`).
Roughly two thirds of what this module offers is invisible to the fragment.
`docs/interface.md` carries them in prose; nothing validates them and
`rammp-deployments` cannot reason about them. To be raised with sheppy
separately — it affects any module that is not a pure topic pipeline.

**The fragment names one image; we have two.** The published image is the sim
build. The KORTEX build links a proprietary aarch64 SDK and cannot go to a
public registry. So the fragment describes the sim configuration, and real-arm
deployment stays on the `make real` path. This is honest but means the declared
contract is not the one that runs on hardware. Revisit when there is a private
registry, or when the SDK's redistribution terms are checked.

## CI

**`lint.yml`** — verbatim. `pre-commit/action@v3.0.1` on push and PR.

**`build.yml`** — three jobs, matching the template:

- `fragment` — `pytest tests`, then `validate_fragment.py rammp-alternative*.yaml`
- `smoke` — `docker build`, then `scripts/smoke.sh` with
  `SMOKE_PATTERN="kinova_gen3_node up"`. The node's startup banner is the
  natural probe, and SIGTERM handling is already correct: the Dockerfile execs
  the node binary directly so it is PID 1 and its handler runs the clean
  `safe_shutdown()` path. `ros2 run` would fork it under a Python wrapper and
  the signal would never arrive.
- `publish` — ghcr, on non-PR events.

**Deviation: `linux/arm64` only.** The template publishes
`linux/arm64,linux/amd64`. The robot is arm64, the pinocchio/cmeel wheels are
pinned to exactly what the Jetson runs, and amd64 has never been built here.
Publishing an untested architecture is a claim we cannot back. Add amd64 when
something needs it and it has been built at least once.

**The KORTEX image is never published.** CI builds and publishes the sim image
only.

## clang-format

Adopting the template's `.clang-format` rewrites every C++ file in the repo.

It lands as a **single commit containing nothing else**, whose SHA is then
added to `.git-blame-ignore-revs`. `git blame` continues to reach the real
authors, and reviewers can verify the commit trivially: it should contain no
semantic change at all.

Sequencing matters — the reformat lands FIRST, before pre-commit is installed,
so that the hook never has a pre-existing violation to fix inside an unrelated
commit.

## Branch model

Adopted from the template's `CONTRIBUTING.md`:

- `main` — demo-ready, stable, deployable. Updated periodically from `dev`.
- `dev` — staging ground. PRs land here.
- `feature/<issue-number>-<brief-description>`, `bug/<issue-number>-<...>`.

`dev` is created from current `main`. Both workflows trigger on `[main, dev]`,
which the template's files already do.

This supersedes the repo's existing ad-hoc `feat/*`, `fix/*`, `chore/*`
convention for new work. Existing merged branches are left alone.

**Branch protection on `main` is a repository setting, not a file.** This spec
does not configure it; it is called out so it is a deliberate decision rather
than an oversight.

**Where this work itself lands.** `dev` does not exist yet, so there is an
ordering question with exactly one sensible answer: create `dev` from the
current `main` as the first step, then this work goes to a
`feature/<issue>-rammp-standards` branch and PRs into `dev`. It would be odd
for the change that adopts the branch model to be the last one to bypass it.

## Verification

The standard's own gates, plus this repo's existing ones — the point is that
adopting the standard must not weaken what we already prove:

- `make lint` — pre-commit clean across the tree.
- `make check` — the fragment validates.
- `make smoke` — the image publishes and exits on SIGTERM.
- `colcon test` — 94 tests, unchanged.
- The conformance suite — 32 passed / 0 failed / 0 skipped under
  `arbitration_mode:=enforced`, unchanged.

The last two are the ones that matter. A deployment-standards change that
alters behaviour has gone wrong.

## Out of scope

- **`tier: integrated`** and `rammp-alternative.mock.yaml`. We are experimental
  until the module has run in a full-system deployment.
- **Fixing sheppy.** `ulimits` translation and warning on untranslated keys are
  `rammp-org/sheppy#10`, not this repo.
- **Releases and versioning.** Separate workstream; `build.yml` already handles
  `type=semver` tags, so tagging will work when the policy exists.
- **The docs site entry.** Adding this repo to `rammp-docs/sources.yml` is the
  docs workstream. `docs/interface.md` is written here because the template
  treats it as part of the contract, not because the site is wired up yet.
