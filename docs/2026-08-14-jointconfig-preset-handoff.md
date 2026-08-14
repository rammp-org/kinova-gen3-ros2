# Handoff — GoToJointConfig + GoToPreset (next implementation session)

**Date:** 2026-08-14
**Purpose:** Carry context into a fresh session to implement the remaining two
high-level joint actions. Read this, then the spec, then the plan, then execute
subagent-driven.

## Read in this order
1. **Plan:** `docs/superpowers/plans/2026-08-14-goto-jointconfig-preset.md` (7 TDD tasks, full code sketches).
2. **Spec:** `docs/superpowers/specs/2026-08-14-goto-jointconfig-preset-design.md` (design + rationale).
3. Prior tier for reference: `docs/superpowers/specs/2026-08-12-...` and the shipped `GoToEEPose` (spec `2026-08-14-goto-ee-pose-curobo-design.md`, plan `2026-08-14-goto-ee-pose.md`).

## Where things stand (branches / PRs)
- **This work's branch: `feat/arm-goto-jointconfig-preset`**, already created and **stacked on `feat/goto-ee-pose-curobo` (PR #2)**. The plan/spec/handoff are committed here; a **draft PR** stacked on PR #2 holds them.
- `GoToEEPose` tier is **PR #2** (`rammp-org/kinova_arm_ros2`), sim-validated, real-arm pending. Its base now includes the RT-core fix `79d1050` (`--cpu 11` pinning + build scoping).
- Core `result_code::kPlanningFailed = -7` is **PR #12** (`rammp-org/kinova-gen3-driver`), not yet merged. **This work reuses it — no new core change.** Merge order: core #12 → PR #2 → this PR.

## Decisions LOCKED (user-approved this session)
1. **Shared lifecycle, full unification.** Extract the `GoToEEPoseServer` plan→execute→settle→cancel lifecycle (the settle-exactly-once machinery) into a templated `PlannedMoveServer<ActionT>` base, and **refactor `GoToEEPose` onto it too** — one audited copy of the concurrency-critical code. The existing `goto_ee_pose_integration_test` is the regression gate (must pass unchanged).
2. **Both new actions plan through cuRobo `plan_to_joints`** (collision-aware) — NOT a planner-bypass direct move. `CuroboPlanClient` gains `plan_to_joints` (+ type-erased `cancel()`).
3. **GoToPreset = a named joint config.** Server resolves `preset_name → 7 joints` from a registry (ROS params `preset_names` + `presets.<name>`, default `home` = cuRobo retract), then plans via `plan_to_joints`.
4. New `.action` Result/Feedback are byte-identical to `GoToEEPose` (so the templated base sets them uniformly). Reuse `kPlanningFailed`.

## Process
Subagent-driven (the loop used all session): brainstorm is DONE (this spec) → the plan is written → execute task-by-task with a fresh implementer per task + per-task review + a final whole-branch review. Tasks 1–7 in the plan; Task 3 (base + GoToEEPose refactor) is the riskiest — its gate is the unchanged GoToEEPose integration test passing.

## Operational facts (don't relearn — mostly carried from the GoToEEPose handoff)
- **Builds aarch64-only on abra.** Dev loop: `bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON`; run a gtest binary via `ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; /tmp/kinova-ros2-ws/build/kinova_arm_ros2/<TESTBIN>"'`.
- **cuRobo interfaces** come from the local clone `~/atdev/RAMMP-CuRobo` (only `rammp_curobo_interfaces` is rsynced/built; `plan_to_joints` is in that same package — no new dep). `FakeCuroboServer` must gain an additive `plan_to_joints` server for tests.
- **RT-core pinning (fix `79d1050`):** real-arm/container runs must pass `--cpu 11` (abra boots `isolcpus=11`); `make sim/real` inject it. No RT-path code changes in this work.
- **Push over HTTPS via gh** (no SSH key this session): `GIT_CONFIG_GLOBAL=/dev/null git -c 'credential.helper=!gh auth git-credential' push https://github.com/rammp-org/kinova_arm_ros2.git <branch>`; `gh pr create --repo … --base feat/goto-ee-pose-curobo --head feat/arm-goto-jointconfig-preset`.
- **Client tooling already on the branch:** `test/send_goto_pose_sequence.py` (samples random EE waypoints in a front-of-arm box, dry-run by default, `--go` to execute). Fold into the tier guide.

## Memory to rely on
`kinova-arm-ros2-repo`, `arm-interface-layer`, `explain-tradeoffs-plainly` (pitch design forks in plain language, not jargon).
