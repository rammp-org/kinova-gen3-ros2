# Contributing

This repo follows the RAMMP module workflow, with the additions a real-time arm
driver needs.

## Pre-commit hooks

Style is enforced automatically before each commit — Python via Ruff, C++ via
clang-format, Dockerfiles via hadolint, workflows via actionlint, plus general file
hygiene.

Run this once after cloning:

```bash
uv tool install pre-commit
pre-commit install
```

Without `uv`: `pip install pre-commit && pre-commit install`.

Hook revisions are pinned. Updating them is a deliberate PR (`pre-commit autoupdate`),
never silent drift.

Two bulk reformats are recorded in `.git-blame-ignore-revs`. Enable it so `git blame`
reaches the author rather than the reformat:

```bash
git config blame.ignoreRevsFile .git-blame-ignore-revs
```

## Branches

- `main` — demo-ready, stable, deployable. Updated periodically from `dev`.
- `dev` — staging ground for tested new code. PRs land here.
- `feature/<issue-number>-<brief-description>` — forked from the latest `dev`. Use
  `bug/<issue-number>-<brief-description>` for fixes.

## Building and testing

Build in a standard ROS 2 Humble colcon workspace (`~/ros2_ws`), as described in the
README's Build section, and run the tests from there:

```bash
cd ~/ros2_ws
colcon build --packages-up-to kinova_gen3_ros2 kinova_gen3_description --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select kinova_gen3_ros2 kinova_gen3_description && colcon test-result --verbose
```

The deployable unit is the container. `make build` builds it; the base image is
published for arm64 only, so run that on an arm64 host.

Two gates, and the second is the one that matters:

- `colcon test` — 94 tests. Unit and integration, no robot.
- **The conformance suite** — 32 checks against a *running* node, asserting the
  behaviour of every ROS control surface rather than printing values to eyeball.
  `32 passed, 0 failed, 0 skipped (mode=enforced)` is the bar.

A change that moves either number has done something, whether or not it meant to.

## The standard's gates

```bash
make lint     # pre-commit run --all-files
make check    # validate the fragments against sheppy itself
make smoke    # does it publish, and does it exit on SIGTERM?
```

`make check` deliberately does not use `rammp-module-template`'s
`validate_fragment.py` — see `scripts/check_fragments.py` for why.

## On-robot work

**Attended only.** Follow `docs/on-robot-runbook.md`: e-stop in hand, and record the
run in that file afterwards.

Claim the cell before driving the arm, and never lift the hardware guard yourself —
it is a human's statement that the cell is attended right now.

Never run the node container with `--rm` on hardware. A crash then destroys the logs
that explain it, which has already cost one attended session.
