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

This module is **two containers**, not one: the driver, and the cuRobo planner
whose `PlanToPose` / `PlanToJoints` actions the `go_to_*` servers are clients
of. Without the planner, three of our four action servers accept goals and
never succeed.

sheppy runs one container per node, and `depends_on` is explicitly ignored
("sheppy owns lifecycle"), so a single fragment cannot express this. We
therefore ship a **compose file describing both services** and two alternatives
that reference it by service name:

```
deploy/compose.yaml          both services, one reviewable file
rammp-alternative.yaml       -> compose: {file: deploy/compose.yaml, service: kinova_gen3_driver}
rammp-alternative.curobo.yaml -> compose: {file: deploy/compose.yaml, service: curobo_planner}
```

### Why the compose-file path rather than an inline `container:`

Two reasons, both load-bearing:

1. **It is the only path with `${VAR}` interpolation.** `load_service()` runs
   `_interpolate`; `DockerLauncher._service()` returns `dict(inline)` verbatim
   for an inline block — still true on the fix branch. Without interpolation
   the arm's IP has to be hardcoded, which bakes one lab's address into the
   module's contract.
2. **It keeps the two containers side by side** in one file a reviewer can
   read, instead of two fragments that drift apart.

```yaml
# deploy/compose.yaml
services:
  kinova_gen3_driver:
    image: kinova-gen3-ros2:kortex          # locally built, see Registry
    network_mode: host
    ipc: host                                # DDS shared-memory transport
    cap_add: [SYS_NICE]
    ulimits: {rtprio: 99, memlock: -1}
    command:
      - /ros2_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node
      - --ip
      - ${KINOVA_ARM_IP:-192.168.1.10}
      - --urdf
      - /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf
      - --cpu
      - ${KINOVA_RT_CORE:-11}

  curobo_planner:
    image: rammp-curobo:jp6                  # locally built, see Registry
    runtime: nvidia                          # JetPack: NOT --gpus
    network_mode: host
    ipc: host
```

`ipc: host` is required for DDS shared-memory transport. `cap_add: SYS_NICE`
permits `sched_setscheduler`; the `ulimits` are what let it succeed —
capabilities and rlimits are separate mechanisms, and `privileged` does NOT
raise `RLIMIT_RTPRIO` (measured: privileged leaves `rtprio` at 0).

`runtime: nvidia` is how JetPack exposes the GPU; `--gpus` is the desktop
spelling and is not what `RAMMP-CuRobo/docker/README.md` documents.

### Dependency on sheppy

`ulimits` and `runtime` are translated on `rammp-org/sheppy`'s
`translate-compose-ulimits` branch (commit `1379b61`, "Translate the whole
compose vocabulary, and stop dropping keys silently"), which also makes an
untranslated key a hard error with a did-you-mean hint rather than a silent
drop. **`origin/main` has neither.** This spec assumes that branch merges;
until it does, both `ulimits` and `runtime` are dropped in silence, which means
a best-effort control loop and a planner with no GPU.

### The template's validator rejects this shape

`validate_fragment.py` requires `container:` to be a mapping and errors
otherwise, so a `compose:`-based fragment fails `make check` even though sheppy
supports the shape. The validator is stricter than the launcher. That is a
template bug to fix there, not a reason to describe a two-container system as
one — to be raised against `rammp-module-template`.

### What the fragment still cannot say

**Actions and services are not expressible.** The schema has `publishes` and
`subscribes` only. This module's primary interface is four action servers and
six services; roughly two thirds of what it offers is invisible to the
fragment, and the driver-to-planner action dependency — the thing that makes
these two containers one system — cannot be declared at all.
`docs/interface.md` carries them in prose; nothing validates them.

### Registry: neither image is published

Both are built on the machine that runs them, and for different reasons:

- **`kinova-gen3-ros2:kortex`** links a proprietary aarch64 SDK. Whether an
  image containing it may be redistributed, even privately within the org, is
  a licence question nobody has answered. **It needs answering before any
  push.**
- **`rammp-curobo:jp6`** is ~15 GB and needs ~25 GB free to build;
  `RAMMP-CuRobo/docker/README.md` specifies building on the target Jetson.

So CI publishes the **sim** image only, and `deploy/compose.yaml` refers to
locally-built tags for both real services. This is the one place the spec
describes something not yet reproducible from a registry, and it is called out
rather than quietly deferred.

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
- **Owning the planner's contract.** `rammp-org/RAMMP-CuRobo` owns its image and
  its actions. `deploy/compose.yaml` references `rammp-curobo:jp6` and pins the
  GPU flags it documents; it does not restate the planner's interface. If the
  planner grows its own fragment, this service should be replaced by a
  reference to it rather than kept in parallel.
- **Fixing sheppy.** `ulimits` translation and warning on untranslated keys are
  `rammp-org/sheppy#10`, not this repo.
- **Releases and versioning.** Separate workstream; `build.yml` already handles
  `type=semver` tags, so tagging will work when the policy exists.
- **The docs site entry.** Adding this repo to `rammp-docs/sources.yml` is the
  docs workstream. `docs/interface.md` is written here because the template
  treats it as part of the contract, not because the site is wired up yet.
