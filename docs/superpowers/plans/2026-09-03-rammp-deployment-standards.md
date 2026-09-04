# RAMMP Deployment Standards Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring `kinova-gen3-ros2` onto the RAMMP module standard — pre-commit, CI with auto-builds, a declared two-container contract, and the interface page the docs site pulls.

**Architecture:** The repo keeps its ROS2 package layout and gains the standard's scaffolding around it. The `Dockerfile` moves to the root so `docker build .` works unmodified. The contract is a compose file describing BOTH containers — the arm driver and the cuRobo planner it calls actions on — referenced by two alternatives, because sheppy runs one container per node and cannot express a dependency.

**Tech Stack:** pre-commit, clang-format, hadolint, actionlint, GitHub Actions, Docker, sheppy.

**Spec:** `docs/superpowers/specs/2026-09-03-rammp-deployment-standards-design.md`

## Global Constraints

- **Behaviour must not change.** The gates that prove it: `colcon test` stays at 94 passing, and the conformance suite stays at `32 passed, 0 failed, 0 skipped (mode=enforced)`. A deployment-standards change that alters either has gone wrong.
- **The clang-format reformat is its own commit**, containing no other change, and its SHA goes in `.git-blame-ignore-revs`.
- **The fragment declares the REAL arm**, not sim. A contract describing a configuration nobody runs is worse than none.
- **`validate_fragment.py` is NOT adopted.** `make check` asks sheppy, which is what actually runs the fragment.
- **`runtime: nvidia`**, never `--gpus` — that is how JetPack exposes the GPU.
- Existing Makefile targets (`sim`, `real`, `e2e`, `build-real`, `stage-kortex`) are KEPT. They encode RT knowledge the standard has no place for.
- Builds and tests run on the remote arm64 machine via `hil.py`. `/tmp/kinova-ros2-ws` there is STALE and not synced — never verify against it.
- Do NOT run `hil.py unlock` or touch the `dojo` guard. If a target is locked, report BLOCKED.
- Commit messages end with:

```
Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01RXbb4Ew5jSLgeT8W33tjLt
```

______________________________________________________________________

### Task 1: Lint configuration, and the isolated reformat

**Files:**

- Create: `.pre-commit-config.yaml`, `.clang-format`, `.hadolint.yaml`, `.git-blame-ignore-revs`
- Modify: every `.cpp`/`.h` under `kinova_gen3_ros2/` (formatting only, second commit)

**Interfaces:**

- Consumes: nothing.

- Produces: a repo where `pre-commit run --all-files` passes.

- [ ] **Step 1: Copy the two configs verbatim**

```bash
cp ~/atdev/rammp-module-template/.pre-commit-config.yaml .
cp ~/atdev/rammp-module-template/.clang-format .
```

Do not edit them. Divergence from the org's style config is exactly what this task exists to remove.

- [ ] **Step 2: Add `.hadolint.yaml` for the three real warnings**

Our Dockerfile produces exactly three, and each has a correct answer rather than a suppression:

```yaml
# hadolint configuration.
#
# DL3008 (pin apt versions): ignored deliberately. The base is `ros:humble`,
# which is itself a moving tag; pinning apt versions underneath a moving base
# gives the appearance of reproducibility without the substance. The pins that
# actually matter here are in docker/requirements.txt, which locks the whole
# cmeel/pinocchio stack to what the Jetson runs.
#
# SC3046 (`source` is undefined in POSIX sh): a false positive. The Dockerfile
# sets SHELL ["/bin/bash", "-c"], so `source` is correct; hadolint's shellcheck
# pass assumes sh regardless.
ignored:
  - DL3008
  - SC3046
```

- [ ] **Step 3: Commit the configs alone — no code changes yet**

```bash
git add .pre-commit-config.yaml .clang-format .hadolint.yaml
git commit -m "chore: adopt the org's pre-commit, clang-format and hadolint config"
```

- [ ] **Step 4: Run the reformat as its own commit**

```bash
uv tool install pre-commit || pip install pre-commit
pre-commit run clang-format --all-files || true   # rewrites files, exits non-zero
git add -A
git commit -m "style: apply the org clang-format

Mechanical. No semantic change. Its own commit so it can be verified by
inspection and skipped by git blame -- see .git-blame-ignore-revs."
```

- [ ] **Step 5: Record the SHA in `.git-blame-ignore-revs`**

```bash
REFORMAT_SHA=$(git rev-parse HEAD)
cat > .git-blame-ignore-revs <<EOF
# Bulk reformats. Ignore with:
#   git config blame.ignoreRevsFile .git-blame-ignore-revs
# (GitHub honours this file automatically.)

# style: apply the org clang-format
$REFORMAT_SHA
EOF
git add .git-blame-ignore-revs
git commit -m "chore: ignore the clang-format reformat in git blame"
```

- [ ] **Step 6: Verify the reformat changed nothing semantic**

Run:

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_gen3_ros2 && nohup make build > /tmp/t1.log 2>&1 & echo started'
# poll to completion:
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'for i in $(seq 1 110); do grep -qE "naming to docker.io|failed to solve|error: " /tmp/t1.log && break; sleep 10; done; grep -E "naming to docker.io|failed to solve|Summary: .*packages finished" /tmp/t1.log | tail -3'
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker run --rm --network host --ipc host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp kinova-gen3-ros2:humble bash -lc "cd /ros2_ws && colcon test --packages-select kinova_gen3_ros2 >/dev/null 2>&1; colcon test-result | tail -2"'
```

Expected: build clean, and `94 tests, 0 errors, 0 failures`. A formatting change that alters a test result is not a formatting change.

- [ ] **Step 7: Verify pre-commit is now clean**

Run: `pre-commit run --all-files`
Expected: every hook passes. If `mdformat` or `end-of-file-fixer` rewrites docs, stage and amend into the config commit — those are not code and do not belong in the reformat commit.

______________________________________________________________________

### Task 2: Dockerfile to the root, and the standard Makefile targets

**Files:**

- Move: `docker/Dockerfile` → `Dockerfile`
- Modify: `Makefile`

**Interfaces:**

- Consumes: nothing from Task 1.

- Produces: `make build|run|check|smoke|lint` targets; `docker build .` works with no `-f`.

- [ ] **Step 1: Move the Dockerfile**

```bash
git mv docker/Dockerfile Dockerfile
```

`docker/requirements.txt` and `docker/vendor/` STAY. The Dockerfile refers to them by paths relative to the build context (the repo root), so `COPY docker/requirements.txt` and `COPY docker/vendor/` keep working unchanged. Only the `-f` flag disappears.

- [ ] **Step 2: Update the Dockerfile's own header comment**

It currently says `docker build -f docker/Dockerfile -t kinova-gen3-ros2:humble .`. Change to `docker build -t kinova-gen3-ros2:humble .`.

- [ ] **Step 3: Update the Makefile's build rules and add the standard targets**

Drop `-f docker/Dockerfile` from `build` and `build-real`. Then add, keeping the existing targets intact:

```make
run:                       ## Run the sim image the way the standard does
	$(RUN) -it $(IMAGE) $(NODE) --sim $(NODE_ARGS)

check:                     ## Validate the fragments against sheppy itself
	uv run --with pyyaml --with sheppy python3 scripts/check_fragments.py \
	  rammp-alternative*.yaml

smoke:                     ## Does it publish, and does it exit on SIGTERM?
	SMOKE_PATTERN="kinova_gen3_node up" ./scripts/smoke.sh $(IMAGE)

lint:                      ## pre-commit run --all-files
	pre-commit run --all-files
```

- [ ] **Step 4: Verify the build still works from the new location**

Run the sync + build + poll sequence from Task 1 Step 6.
Expected: `Summary: 4 packages finished`, image `kinova-gen3-ros2:humble`.

- [ ] **Step 5: Commit**

```bash
git add Dockerfile Makefile
git commit -m "chore: Dockerfile to the repo root, plus the standard make targets"
```

______________________________________________________________________

### Task 3: The two-container contract

**Files:**

- Create: `deploy/compose.yaml`, `rammp-alternative.yaml`, `rammp-alternative.curobo.yaml`, `scripts/check_fragments.py`

**Interfaces:**

- Consumes: the `check` target from Task 2.

- Produces: fragments that `make check` validates.

- [ ] **Step 1: Write `deploy/compose.yaml`**

```yaml
# Both containers of this module, in one file.
#
# This module is TWO containers: the arm driver, and the cuRobo planner whose
# PlanToPose/PlanToJoints actions the go_to_* servers are clients of. Without
# the planner, three of the four action servers accept goals and never succeed.
#
# sheppy runs one container per node and ignores depends_on ("sheppy owns
# lifecycle"), so this cannot be one fragment. Two alternatives reference this
# file by service name.
#
# The compose-file path is also the ONLY one with ${VAR} interpolation:
# load_service() calls _interpolate, while an inline `container:` block is
# returned verbatim. That is what gives the arm's IP somewhere to live.
services:
  kinova_gen3_driver:
    image: kinova-gen3-ros2:kortex
    network_mode: host
    ipc: host                      # DDS shared-memory transport
    cap_add: [SYS_NICE]            # permits sched_setscheduler...
    ulimits:                       # ...and these are what let it SUCCEED.
      rtprio: 99                   # privileged does NOT raise RLIMIT_RTPRIO
      memlock: -1
    command:
      - /ros2_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node
      - --ip
      - ${KINOVA_ARM_IP:-192.168.1.10}
      - --urdf
      - /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf
      - --cpu
      - ${KINOVA_RT_CORE:-11}

  curobo_planner:
    image: rammp-curobo:jp6
    runtime: nvidia                # JetPack exposes the GPU this way, NOT --gpus
    network_mode: host
    ipc: host
```

- [ ] **Step 2: Write the two fragments**

`rammp-alternative.yaml`:

```yaml
# The arm driver. See deploy/compose.yaml for the container definition, and
# docs/interface.md for the actions and services this fragment cannot express.
id: kinova_gen3_driver
tier: experimental
kind: docker
ros_node_name: kinova_gen3_node
compose:
  file: deploy/compose.yaml
  service: kinova_gen3_driver
params:
  arbitration_mode: disabled
  estop_clear_max_age_s: 1.0
  expect_gripper: true
publishes:
  - /joint_states
  - /ee_state
  - /control_status
  - /stream_status
  - /gripper_state
  - /diagnostics
subscribes:
  - /estop
  - /setpoint/joint_position
  - /setpoint/joint_velocity
  - /setpoint/joint_torque
  - /setpoint/pose
  - /setpoint/twist
  - /setpoint/wrench
  - /setpoint/gripper
```

`rammp-alternative.curobo.yaml`:

```yaml
# The planner this module calls. Owned by rammp-org/RAMMP-CuRobo; declared here
# only because sheppy cannot express "and also start that one", and running the
# driver without it leaves three action servers permanently unsatisfied.
#
# If the planner grows its own fragment, replace this with a reference to it.
id: curobo_planner
tier: experimental
kind: docker
compose:
  file: deploy/compose.yaml
  service: curobo_planner
publishes: []
subscribes: []
```

- [ ] **Step 3: Write `scripts/check_fragments.py`**

```python
#!/usr/bin/env python3
"""Validate our fragments against sheppy -- the thing that will run them.

We deliberately do NOT use rammp-module-template's validate_fragment.py. It
requires a `container:` mapping, so it rejects the compose: shape sheppy
supports, and past that it checks only that container.image is a non-empty
string. Too strict and too loose at once, and copied into every module repo so
each copy drifts as sheppy's vocabulary grows.

This reaches into sheppy's launcher internals, which is not ideal -- it is the
interim until `sheppy validate` exists (rammp-org/sheppy#14).
"""
import sys

import yaml
from sheppy.launch.docker import DockerLauncher
from sheppy.launch.docker.compose import load_service, service_to_docker_args


def check(path: str) -> list:
    with open(path) as f:
        frag = yaml.safe_load(f) or {}
    errors = list(DockerLauncher().validate(frag) or [])

    # validate() checks the fragment's shape; the compose SERVICE it points at
    # is only translated at launch, so translate it here too or a bad key
    # inside the service would sail through.
    ref = frag.get("compose") or {}
    if ref.get("file") and ref.get("service"):
        try:
            service = load_service(ref["file"], ref["service"], {})
        except (OSError, KeyError) as e:
            return errors + [f"cannot load {ref['file']}#{ref['service']}: {e}"]
        _, _, _, errs, warns = service_to_docker_args(service)
        errors += list(errs)
        for w in warns:
            print(f"  {path}: warning: {w}")
    return errors


def main() -> int:
    failed = False
    for path in sys.argv[1:]:
        errs = check(path)
        if errs:
            failed = True
            for e in errs:
                print(f"  {path}: {e}")
        else:
            print(f"  {path}: ok")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run it and confirm both fragments validate**

Run: `make check`
Expected: `rammp-alternative.yaml: ok` and `rammp-alternative.curobo.yaml: ok`, exit 0.

If sheppy rejects a key, that is a real finding — report it, do NOT delete the key to go green. The whole point of validating against sheppy is that it tells the truth about what will run.

- [ ] **Step 5: Commit**

```bash
git add deploy/compose.yaml rammp-alternative.yaml rammp-alternative.curobo.yaml scripts/check_fragments.py
git commit -m "feat: declare the contract -- both containers, validated by sheppy"
```

______________________________________________________________________

### Task 4: Smoke test and CI

**Files:**

- Create: `scripts/smoke.sh`, `.github/workflows/lint.yml`, `.github/workflows/build.yml`

**Interfaces:**

- Consumes: `Dockerfile` at root (Task 2), `make check` (Task 3).

- Produces: CI that lints, validates, smokes and publishes.

- [ ] **Step 1: Copy and adapt `smoke.sh`**

```bash
cp ~/atdev/rammp-module-template/scripts/smoke.sh scripts/smoke.sh
```

Change only the `PATTERN` default and the default image:

```bash
PATTERN="${SMOKE_PATTERN:-kinova_gen3_node up}"
IMAGE="${1:-kinova-gen3-ros2:humble}"
```

The rest is correct as-is. Note what it already proves for us: the node must exit on SIGTERM, and our Dockerfile execs the node binary directly so it is PID 1 and its handler runs `safe_shutdown()`. `ros2 run` would fork it under a Python wrapper and the signal would never arrive — the smoke test would catch that regression.

`smoke` runs the SIM image, not the KORTEX one. It is the hardware-free path by design; the fragment declaring the real arm does not change what CI can run on a machine with no arm.

- [ ] **Step 2: Copy `lint.yml` verbatim**

```bash
mkdir -p .github/workflows
cp ~/atdev/rammp-module-template/.github/workflows/lint.yml .github/workflows/lint.yml
```

No changes. It triggers on `[main, dev]`, which Task 5 makes true.

- [ ] **Step 3: Write `build.yml`**

Start from the template's and make exactly the deviations the spec calls for:

```yaml
name: Build and Publish

on:
  push:
    branches: [main, dev]
    tags: ["v*"]
  pull_request:
    branches: [main, dev]
  workflow_dispatch:

permissions:
  contents: read
  packages: write

jobs:
  fragment:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.11"
      # Validated against sheppy itself, not a copied script -- see
      # scripts/check_fragments.py for why.
      - name: Install sheppy
        run: pip install pyyaml git+https://github.com/rammp-org/sheppy@main
      - name: Validate the fragments
        run: python3 scripts/check_fragments.py rammp-alternative*.yaml

  smoke:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - name: Build the sim image
        run: docker build -t kinova-gen3-ros2:ci .
      - name: Publishes, and exits on SIGTERM
        run: SMOKE_PATTERN="kinova_gen3_node up" ./scripts/smoke.sh kinova-gen3-ros2:ci

  publish:
    needs: [fragment, smoke]
    if: github.event_name != 'pull_request'
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: docker/setup-qemu-action@v3
      - uses: docker/setup-buildx-action@v3
      - uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - id: image
        run: echo "name=${GITHUB_REPOSITORY,,}" >> "$GITHUB_OUTPUT"
      - id: meta
        uses: docker/metadata-action@v5
        with:
          images: ghcr.io/${{ steps.image.outputs.name }}
          tags: |
            type=ref,event=branch
            type=semver,pattern={{version}}
            type=sha,format=long
      # Both arches. amd64 was verified on 2026-09-03: it builds (cmeel ships
      # manylinux_2_28_x86_64 wheels), the node starts, and conformance passes
      # 32/32 -- the same numbers as arm64. arm64 is what the robot needs and is
      # emulated under QEMU here, which is the long pole either way, so amd64
      # costs little on top. The KORTEX image is never published: proprietary SDK.
      - uses: docker/build-push-action@v6
        with:
          context: .
          platforms: linux/arm64,linux/amd64
          push: true
          tags: ${{ steps.meta.outputs.tags }}
          labels: ${{ steps.meta.outputs.labels }}
```

- [ ] **Step 4: Verify the smoke test locally before trusting CI**

Run:

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_gen3_ros2 && SMOKE_PATTERN="kinova_gen3_node up" ./scripts/smoke.sh kinova-gen3-ros2:humble'
```

Expected: passes — the log matches, the container is still running, and it exits on SIGTERM.

- [ ] **Step 5: Commit**

```bash
git add scripts/smoke.sh .github/workflows/
git commit -m "ci: lint, fragment validation, smoke and an arm64-only publish"
```

______________________________________________________________________

### Task 5: The interface page and the branch model

**Files:**

- Create: `docs/interface.md`, `CONTRIBUTING.md`

**Interfaces:**

- Consumes: the fragments from Task 3.

- Produces: the page `rammp-docs` pulls, and the documented workflow.

- [ ] **Step 1: Write `docs/interface.md`**

Keep the template's section headings — `rammp-docs` pulls this file and they are load-bearing. Fill them from this repo's actual surface. It MUST carry what the fragment cannot express: the four action servers, the six services, and the fact that `go_to_*` requires the planner.

Cover, in the template's heading order: what it does; **Publishes** (the six topics with types and rates); **Subscribes** (the eight, with the note that setpoints apply only inside an open stream session with a valid token); **Actions** and **Services** as added sections, since the template has no slot for them and they are two thirds of this interface; **Parameters** (`arbitration_mode` read-only at launch, `estop_clear_max_age_s`, `expect_gripper`); and the two-container requirement.

State plainly that gripper `velocity`/`effort` are NaN in `/joint_states` by design — `sensor_msgs` documents effort in N·m and core's is a 0..1 current fraction — and that a sustained grasp reports a SMALL effort (~0.05).

- [ ] **Step 2: Write `CONTRIBUTING.md`**

Adapt the template's. Keep the branch model verbatim (`main` / `dev` / `feature/<issue>-<desc>`, `bug/<issue>-<desc>`) and the pre-commit setup instructions. Add this repo's specifics: builds happen on the arm64 machine through `hil.py`, the conformance suite is the real gate, and on-arm changes are attended per `docs/on-robot-runbook.md`.

- [ ] **Step 3: Create `dev` from `main`**

```bash
git checkout main && git pull
git checkout -b dev
git push -u origin dev
```

This must exist before the workflows' `[main, dev]` triggers mean anything.

- [ ] **Step 4: Commit**

```bash
git add docs/interface.md CONTRIBUTING.md
git commit -m "docs: the interface page rammp-docs pulls, and the branch model"
```

______________________________________________________________________

## Final verification

- [ ] **The standard's gates**

```bash
make lint     # pre-commit clean
make check    # both fragments validate against sheppy
make smoke    # publishes, exits on SIGTERM
```

- [ ] **This repo's gates — the ones that matter**

Build, then `colcon test`, then the conformance suite in sim under `arbitration_mode:=enforced`.
Expected: `94 tests, 0 errors, 0 failures` and `32 passed, 0 failed, 0 skipped`.

Unchanged from before this work. A deployment-standards change that moves either number has gone wrong, and the number to trust is the conformance one.
