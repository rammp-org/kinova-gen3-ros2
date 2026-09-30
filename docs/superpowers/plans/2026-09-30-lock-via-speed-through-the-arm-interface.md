# Locks, Approach Via Points and Speed Through the Arm Interface — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `GoToEEPose` goal can ask for a locked tool orientation and a blended approach, and every planned move can ask to run slower — and all of it survives the trip from the ROS action, through the node, to the planner and the driver.

**Architecture:** The arm contract defines its **own** lock and approach messages rather than embedding the planner's, so replacing cuRobo is not a breaking change for every arm client. `CuroboPlanClient` — the only unit that knows cuRobo exists — translates arm types into planner types on the way out. `speed_scale` needs no translation: it rides on `TrajectoryGoal` straight to the driver, which dilates its executor clock.

**Tech Stack:** ROS 2 Humble, rosidl, C++17, GoogleTest, colcon, Cyclone DDS.

**Spec:** `rammp-org/kinova-gen3-ros2#38`. Read it first — it carries the two decisions this plan assumes (arm-owned message types, and the via-point-implies-holding semantic found on hardware).

**Two repositories.** `rammp-interfaces-ros2` (worktree `/home/swapnil/atdev/rammp-interfaces-lockvia`, branch `feature/38-lock-via-speed`) and `kinova-gen3-ros2` (worktree `/home/swapnil/atdev/kinova-ros2-lockvia`, same branch name). Task 1 is the first; Tasks 2-5 the second. Both are already staged on the build host.

## Global Constraints

- **Append only, at the very end, with a default.** On Humble a subscriber matches a publisher on **type name alone** — type hashes arrived in Iron — and the deserialiser leaves an absent trailing field at its declared default. So appending is a MINOR bump; inserting anywhere else silently corrupts every field after it and is MAJOR under any RMW. This holds only while every module runs **Cyclone DDS**.
- **A nested message field cannot carry a default in the `.action`**, so "off" must be what a default-constructed message already means: no locks, zero offset.
- **Planner types must not leak.** `rammp_curobo_interfaces` may be named only inside `CuroboPlanClient` (its header says it is "the ONLY unit that knows cuRobo exists"). Servers, `message_mapping` and the actions deal exclusively in arm types.
- **The node validates and explains; the driver validates and refuses.** Both check the same ranges. The driver's `GoalResponse` carries no message, so the *reason* must come from the node.
- A goal that sets none of the new fields must behave **exactly** as before. The existing 93-test suite is the regression gate; if it needs editing, something changed that should not have.

## Build and test — on abra, in the released node image

There is no ROS toolchain on this laptop, and the node's own image builds `FROM rammp-base:1.0.0-jp6` (arm64/JetPack only) while its `CORE_REF` override fetches the driver **from the remote** — which cannot reach these unpushed branches. Instead, all four sources are mounted into the released node image, which already carries Pinocchio, ROS 2 Humble and colcon.

```sh
bash /home/swapnil/atdev/.driver-deps/sync-lockvia.sh    # push the 4 worktrees to abra
bash /home/swapnil/atdev/.driver-deps/build-lockvia.sh   # colcon build + colcon test, prints results
```

**Verified baseline before this plan: 5 packages build, 93 tests, 0 failures.** Re-sync after every edit — the build reads the copy on abra, not your worktree. `abra` is a **shared cell**: do not start the arm driver, do not run any `sheppy` command, do not command the arm. Building and running unit tests is all this plan needs.

## Review Focus

Five things the spec implies that no task's happy path exercises.

1. **An older client must be bit-for-bit unaffected.** Every new field is appended with a default, but the *mapping* must also treat a default-constructed message as "off" rather than as a request for something. → Task 4, Step 9.
2. **A lock that contradicts the approach.** cuRobo frees the approach axis; a caller that also locks that axis is asking for two opposite things. The planner refuses it — the node should say so first. → Task 4, Step 7.
3. **Out-of-range speed at the ROS boundary.** The driver refuses without a message. A client that sends 0.0, 1.5 or NaN must get a sentence, not a bare rejection. → Task 2, Step 5.
4. **Joint-space goals carry speed but not locks.** `GoToJointConfig` and `GoToPreset` plan in joint space, where a tool-pose lock has no meaning. Confirm they carry `speed_scale` and nothing else, and that this is stated rather than implied. → Task 4, Step 11.
5. **The translation must not silently drop a field.** Arm lock → planner lock is six booleans and a frame; a mis-mapped index constrains the wrong axis and still plans. → Task 3, Step 5.

---

### Task 1: The arm contract

**Files (in `/home/swapnil/atdev/rammp-interfaces-lockvia`):**
- Create: `rammp_arm_interfaces/msg/ToolAxisLock.msg`, `rammp_arm_interfaces/msg/ApproachOffset.msg`
- Modify: `rammp_arm_interfaces/CMakeLists.txt` (registration), `rammp_arm_interfaces/package.xml` (version), the four `.action` files, `README.md`

**Interfaces:**
- Consumes: nothing.
- Produces: `rammp_arm_interfaces/msg/ToolAxisLock`, `rammp_arm_interfaces/msg/ApproachOffset`, and the appended goal fields `speed_scale` (all four actions), `axis_lock` + `approach_offset` (`GoToEEPose` only).

**Naming note for the reviewer:** these are deliberately *not* called `PoseAxisLock`/`ApproachVia` like the planner's messages. Distinct names keep the translation boundary visible — the arm contract and the planner contract are different things that happen to be isomorphic today.

- [ ] **Step 1: Write `ToolAxisLock.msg`**

```
# Tool-pose components to hold fixed for the whole trajectory.
#
# A locked component is held AT THE GOAL'S VALUE, so the start pose must already
# match the goal on that component. "Locked" means unchanged, not level: a tool
# at 45 degrees planning to a 45-degree goal stays at 45 the whole way.
#
# All fields false (the default) means unconstrained.

uint8 FRAME_BASE=0    # the robot base frame — gravity-relative, what "level" means
uint8 FRAME_GOAL=1    # projected into the goal frame
uint8 reference_frame 0

bool lock_roll
bool lock_pitch
bool lock_yaw
bool lock_x
bool lock_y
bool lock_z
```

- [ ] **Step 2: Write `ApproachOffset.msg`**

```
# Approach the goal along one tool axis, through a blended intermediate target.
#
# IMPORTANT: an approach is not independent of locking. The planner holds the
# OTHER FIVE pose components at the goal's values while travelling along the
# freed axis, so requesting an approach also requests those five locks — and
# therefore requires the start to already match the goal on them. Verified
# against cuRobo v0.7.8 on 2026-09-26.
#
# It is also a preference, not a waypoint: the path passes NEAR the offset
# without stopping there and without hitting it exactly.
#
# distance of 0.0 (the default) means no approach offset.

uint8 AXIS_X=0
uint8 AXIS_Y=1
uint8 AXIS_Z=2        # the tool approach axis — the usual choice

float64 distance      # metres back along `axis` from the goal
uint8 axis 2
float64 at_fraction 0.8   # fraction of the motion at which it engages, in (0, 1)
```

- [ ] **Step 3: Register both in `CMakeLists.txt`**

Add `"msg/ToolAxisLock.msg"` and `"msg/ApproachOffset.msg"` to the existing `rosidl_generate_interfaces` list, beside the other `msg/` entries. The `DEPENDENCIES` line already carries everything these need.

- [ ] **Step 4: Append the fields to the actions**

To **all four** of `ExecuteJointTrajectory.action`, `GoToEEPose.action`, `GoToJointConfig.action`, `GoToPreset.action`, at the very end of the **goal** block (immediately before the first `---`):

```
# --- appended 2026-09-30. Execute this move slower: (0, 1], 1.0 = as planned.
# Out of range is refused, not clamped. Appended-at-the-end-with-a-default is a
# MINOR bump; see this repo's README.
float64 speed_scale 1.0
```

And to `GoToEEPose.action` **only**, after that line:

```
# Hold tool-pose components fixed for the whole trajectory; all-false = free.
ToolAxisLock axis_lock
# Approach the goal along one tool axis. NOTE: an approach also holds the other
# five pose components — see ApproachOffset.msg. distance 0.0 = no approach.
ApproachOffset approach_offset
```

Do **not** add these two to the joint-space actions: `GoToJointConfig` and `GoToPreset` plan in joint space, where a tool-pose lock has no meaning.

- [ ] **Step 5: Bump the version and note it**

`rammp_arm_interfaces/package.xml`: bump the **minor** version. Add a line to `README.md`'s version history (or CHANGELOG if the repo has one) naming the three appended fields and the two new messages, and stating that it is minor because all three are appended at the end with defaults.

- [ ] **Step 6: Build and confirm the generated goal's field order**

Re-sync and build:

```sh
bash /home/swapnil/atdev/.driver-deps/sync-lockvia.sh
bash /home/swapnil/atdev/.driver-deps/build-lockvia.sh
```

Then check the generated type directly on abra — this is the step that proves the bump is really minor:

```sh
ssh abra 'docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -v /home/abra/lockvia-ws:/ws -w /ws \
  ghcr.io/rammp-org/kinova-gen3-ros2:1.0.0 bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && \
  python3 -c \"
from rammp_arm_interfaces.action import GoToEEPose
g = GoToEEPose.Goal()
print(list(g.get_fields_and_field_types()))
print(g.speed_scale, g.approach_offset.distance, g.approach_offset.axis, g.approach_offset.at_fraction)
print(any([g.axis_lock.lock_roll, g.axis_lock.lock_pitch, g.axis_lock.lock_yaw,
           g.axis_lock.lock_x, g.axis_lock.lock_y, g.axis_lock.lock_z]))
\""'
```

Expected: the three new fields appear **last**, in the order added; `speed_scale` is 1.0; `distance` is 0.0 with `axis` 2 and `at_fraction` 0.8; and the lock reports `False` — a default goal is inert. Paste the output into your report.

- [ ] **Step 7: Commit**

```bash
git -C /home/swapnil/atdev/rammp-interfaces-lockvia add rammp_arm_interfaces README.md
git -C /home/swapnil/atdev/rammp-interfaces-lockvia commit -m "feat(arm): tool axis lock, approach offset and speed_scale on the GoTo actions

Arm-owned message types rather than the planner's, so replacing cuRobo is
not a breaking change for every arm client. All fields appended at the end
with defaults: a client that sets none behaves exactly as before."
```

---

### Task 2: Carry `speed_scale` to the driver

**Files (in `/home/swapnil/atdev/kinova-ros2-lockvia`):**
- Modify: `kinova_gen3_ros2/src/message_mapping.cpp`, `kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h`
- Test: `kinova_gen3_ros2/test/message_mapping_test.cpp`

**Interfaces:**
- Consumes: Task 1's `speed_scale`; `kinova::interface::TrajectoryGoal::speed_scale` from the driver.
- Produces: `speed_scale` set on both `to_trajectory_goal` overloads, and a free function `std::optional<std::string> speed_scale_rejection(double)` returning a reason when out of range.

- [ ] **Step 1: Write the failing tests**

Append to `kinova_gen3_ros2/test/message_mapping_test.cpp`:

```cpp
TEST(MessageMapping, SpeedScaleReachesTheTrajectoryGoal) {
  rammp_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points.resize(1);
  g.trajectory.points[0].positions.assign(7, 0.0);
  g.speed_scale = 0.25;
  const auto tg = to_trajectory_goal(g);
  EXPECT_DOUBLE_EQ(tg.speed_scale, 0.25);
}

TEST(MessageMapping, ADefaultGoalIsFullSpeed) {
  // The whole point of appending with a default: a client that knows nothing
  // about speed_scale must plan and execute exactly as it did before.
  rammp_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points.resize(1);
  g.trajectory.points[0].positions.assign(7, 0.0);
  const auto tg = to_trajectory_goal(g);
  EXPECT_DOUBLE_EQ(tg.speed_scale, 1.0);
}

TEST(MessageMapping, SpeedScaleRejectionNamesTheProblem) {
  EXPECT_FALSE(speed_scale_rejection(1.0).has_value());
  EXPECT_FALSE(speed_scale_rejection(0.01).has_value());
  for (double bad : {0.0, -0.5, 1.5, std::numeric_limits<double>::quiet_NaN()}) {
    const auto why = speed_scale_rejection(bad);
    ASSERT_TRUE(why.has_value()) << "scale " << bad << " must be refused";
    EXPECT_NE(why->find("speed_scale"), std::string::npos)
        << "the reason must name the field the client got wrong";
  }
}
```

Add `#include <limits>` and `#include <optional>` to that file if absent.

- [ ] **Step 2: Run them and watch them fail**

```sh
bash /home/swapnil/atdev/.driver-deps/sync-lockvia.sh && bash /home/swapnil/atdev/.driver-deps/build-lockvia.sh
```
Expected: compile error — `speed_scale_rejection` undeclared, and `tg.speed_scale` unset.

- [ ] **Step 3: Implement**

In `message_mapping.h`, declare beside the other free functions:

```cpp
// Why this speed_scale is unacceptable, or nullopt if it is fine. The driver
// refuses the same range but its GoalResponse carries no message, so the
// explanation has to come from here.
std::optional<std::string> speed_scale_rejection(double s);
```

In `message_mapping.cpp`:

```cpp
std::optional<std::string> speed_scale_rejection(double s) {
  if (!std::isfinite(s)) return std::string("speed_scale must be finite");
  if (s <= 0.0 || s > 1.0)
    return "speed_scale must be in (0, 1]; got " + std::to_string(s);
  return std::nullopt;
}
```

and in **both** `to_trajectory_goal` overloads, set `tg.speed_scale = g.speed_scale;` — for the planner overload (which takes a bare `JointTrajectory` and has no goal to read), add a `double speed_scale = 1.0` trailing parameter and set it from that, so the caller in `PlannedMoveServer` can pass the goal's value through.

- [ ] **Step 4: Run and confirm**

Re-sync and build. Expected: the three new tests pass and the pre-existing 93 still pass.

- [ ] **Step 5: Commit**

```bash
git -C /home/swapnil/atdev/kinova-ros2-lockvia add kinova_gen3_ros2/src/message_mapping.cpp kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h kinova_gen3_ros2/test/message_mapping_test.cpp
git -C /home/swapnil/atdev/kinova-ros2-lockvia commit -m "feat(ros): carry speed_scale onto TrajectoryGoal

The driver refuses an out-of-range scale without a message; this layer
refuses it with one."
```

---

### Task 3: Translate arm lock and approach into planner types

**Files:**
- Modify: `kinova_gen3_ros2/include/kinova_gen3_ros2/curobo_plan_client.h`, `kinova_gen3_ros2/src/curobo_plan_client.cpp`
- Test: `kinova_gen3_ros2/test/curobo_plan_client_test.cpp`, `kinova_gen3_ros2/test/fake_curobo_server.h`

**Interfaces:**
- Consumes: Task 1's `ToolAxisLock` / `ApproachOffset`.
- Produces: `CuroboPlanClient::plan(target, start_joints, axis_lock, approach_offset, on_fb, on_done)`; the existing four-argument `plan(...)` stays, delegating with default-constructed arm messages; `FakeCuroboServer::last_axis_lock()` / `last_approach_via()` accessors.

**This is the only unit allowed to name `rammp_curobo_interfaces`.** Its header says so. The translation lives here and nowhere else.

- [ ] **Step 1: Teach the fake to record what it received**

In `kinova_gen3_ros2/test/fake_curobo_server.h`, record the incoming `axis_lock` and `approach_via` from the `PlanToPose` goal, and expose them the way `last_start_joints()` already is. Without this the translation cannot be observed at all.

- [ ] **Step 2: Write the failing test**

```cpp
TEST(CuroboPlanClient, TranslatesTheArmLockOntoThePlannerLock) {
  // Six booleans and a frame. A mis-mapped index constrains the WRONG axis and
  // still plans, so assert the mapping element by element rather than "some
  // lock arrived".
  //
  // Build the node, callback group, FakeCuroboServer, CuroboPlanClient and the
  // spin helper exactly as the first test in this file does — copy that setup
  // verbatim rather than inventing a second harness.
  rammp_arm_interfaces::msg::ToolAxisLock lock;
  lock.lock_roll = true;
  lock.lock_z = true;
  lock.reference_frame = rammp_arm_interfaces::msg::ToolAxisLock::FRAME_BASE;
  client.plan(pose, start, lock, {}, on_fb, on_done);
  // Spin until the fake reports it has been called, the way the existing tests
  // in this file wait on their `started` promise.
  const auto got = fake.last_axis_lock();
  EXPECT_TRUE(got.lock_roll);
  EXPECT_FALSE(got.lock_pitch);
  EXPECT_FALSE(got.lock_yaw);
  EXPECT_FALSE(got.lock_x);
  EXPECT_FALSE(got.lock_y);
  EXPECT_TRUE(got.lock_z);
  EXPECT_EQ(got.reference_frame,
            rammp_curobo_interfaces::msg::PoseAxisLock::FRAME_BASE);
}

TEST(CuroboPlanClient, TranslatesTheApproachOffset) {
  // Same setup as the test above.
  rammp_arm_interfaces::msg::ApproachOffset off;
  off.distance = 0.10;
  off.axis = rammp_arm_interfaces::msg::ApproachOffset::AXIS_Z;
  off.at_fraction = 0.7;
  client.plan(pose, start, {}, off, on_fb, on_done);
  const auto got = fake.last_approach_via();
  EXPECT_DOUBLE_EQ(got.offset, 0.10);
  EXPECT_EQ(got.axis, rammp_curobo_interfaces::msg::ApproachVia::AXIS_Z);
  EXPECT_DOUBLE_EQ(got.at_fraction, 0.7);
}

TEST(CuroboPlanClient, TheFourArgumentPlanSendsNothingExtra) {
  // The pre-existing call site must keep behaving exactly as before.
  client.plan(pose, start, on_fb, on_done);
  const auto lock = fake.last_axis_lock();
  EXPECT_FALSE(lock.lock_roll || lock.lock_pitch || lock.lock_yaw ||
               lock.lock_x || lock.lock_y || lock.lock_z);
  EXPECT_DOUBLE_EQ(fake.last_approach_via().offset, 0.0);
}
```

Follow the file's existing patterns for constructing the client, the fake and the spin — read them rather than inventing a new harness.

- [ ] **Step 3: Run and watch it fail**

Re-sync and build. Expected: compile error on the six-argument `plan`.

- [ ] **Step 4: Implement the overload and the translation**

Add to `curobo_plan_client.h` a six-argument `plan(...)` taking `const rammp_arm_interfaces::msg::ToolAxisLock&` and `const rammp_arm_interfaces::msg::ApproachOffset&` before the callbacks, and keep the four-argument form delegating to it with default-constructed messages. In the `.cpp`, add two file-static translation functions — arm lock → `rammp_curobo_interfaces::msg::PoseAxisLock`, arm offset → `ApproachVia` — mapping the six booleans one by one, the frame constant, and `distance`/`axis`/`at_fraction` onto `offset`/`axis`/`at_fraction`.

Map the frame constants **explicitly** (`FRAME_BASE` → `FRAME_BASE`, `FRAME_GOAL` → `FRAME_GOAL`), not by assigning the raw integer: the two enumerations are independent contracts that happen to agree today.

- [ ] **Step 5: Run and confirm**

Re-sync and build. Expected: the three new tests pass; the pre-existing suite unchanged.

- [ ] **Step 6: Commit**

```bash
git -C /home/swapnil/atdev/kinova-ros2-lockvia add kinova_gen3_ros2/include/kinova_gen3_ros2/curobo_plan_client.h kinova_gen3_ros2/src/curobo_plan_client.cpp kinova_gen3_ros2/test/
git -C /home/swapnil/atdev/kinova-ros2-lockvia commit -m "feat(ros): translate the arm's lock and approach into planner types

The translation lives in CuroboPlanClient, the only unit allowed to name
rammp_curobo_interfaces. Frame constants are mapped explicitly: the two
enumerations are independent contracts that happen to agree today."
```

---

### Task 4: Wire the servers

**Files:**
- Modify: `kinova_gen3_ros2/include/kinova_gen3_ros2/goto_ee_pose_server.h`, `goto_joint_config_server.h`, `goto_preset_server.h`, `planned_move_server.h`
- Test: `kinova_gen3_ros2/test/goto_ee_pose_integration_test.cpp`, and the joint-config/preset integration tests

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: the three servers passing their new fields through; validation at accept time with a reason.

- [ ] **Step 1: Write the failing integration tests**

In `goto_ee_pose_integration_test.cpp`, following the file's existing fixture:

Three tests, each built on the fixture this file already uses for
`PlanRequestCarriesTheMeasuredStartConfiguration` — copy that test's setup and
change only the goal:

- `ALockedGoalReachesThePlanner` — set `goal.axis_lock.lock_roll = true` and
  `lock_pitch = true`; assert via `fake.last_axis_lock()` that exactly those two
  arrive true and the other four false.
- `AnApproachOffsetReachesThePlanner` — set `goal.approach_offset.distance = 0.10`;
  assert `fake.last_approach_via().offset` is 0.10 and the axis is Z.
- `SpeedScaleReachesTheTrajectoryGoal` — set `goal.speed_scale = 0.5`; assert
  `sup.last_goal.speed_scale` is 0.5. `FakeSupervisor` in
  `planned_move_test_fixture.h` already records `last_goal` — read it rather than
  adding a new recorder.

- [ ] **Step 2: Run and watch them fail**

- [ ] **Step 3: Pass the fields through in `GoToEEPoseServer::start_plan`**

```cpp
  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan(goal.target.pose, this->start_config(), goal.axis_lock,
                  goal.approach_offset, std::move(on_fb), std::move(on_done));
  }
```

- [ ] **Step 4: Carry `speed_scale` from the goal onto the planned `TrajectoryGoal`**

In `planned_move_server.h`'s `on_plan_done`, where `to_trajectory_goal(outcome.trajectory)` is called, pass the goal's `speed_scale` through the new trailing parameter from Task 2. The goal handle is in scope there; read it from `gh->get_goal()->speed_scale`.

- [ ] **Step 5: Run and confirm**

- [ ] **Step 6: Commit**

- [ ] **Step 7: Add the contradiction test and check (Review Focus 2)**

A caller that locks the same axis the approach travels along is asking for two opposite things. Refuse it in `GoToEEPoseServer::validate` with a reason naming both fields, and test it. Map the axis to the lock it contradicts: `AXIS_X`→`lock_x`, `AXIS_Y`→`lock_y`, `AXIS_Z`→`lock_z`, and only when `distance != 0.0`.

- [ ] **Step 8: Add speed validation at accept time (Review Focus 3)**

In each of the three servers' `validate`, call `speed_scale_rejection(goal.speed_scale)` and return the reason. Test with 0.0, 1.5 and NaN on at least one server; the shared helper covers the arithmetic.

- [ ] **Step 9: Add the unchanged-client test (Review Focus 1)**

A `GoToEEPose` goal with none of the new fields set must produce a plan request with no lock and no offset, and a `TrajectoryGoal` at `speed_scale == 1.0`. Assert all three in one test, so the "nothing changed for existing clients" claim is pinned in one readable place.

- [ ] **Step 10: Run the full suite and commit**

- [ ] **Step 11: Wire and test speed on the joint-space servers (Review Focus 4)**

`GoToJointConfigServer` and `GoToPresetServer` carry `speed_scale` and nothing else. Add the validation call and one test each asserting the scale reaches the `TrajectoryGoal`. Add a comment in each saying locks and approaches are deliberately absent because these plan in joint space.

- [ ] **Step 12: Run the full suite and commit**

---

### Task 5: Correct the documentation, including a rule that is stated wrongly

**Files:**
- Modify: `kinova_gen3.repos`, `docs/guide-goto-ee-pose.md`, `docs/guide-goto-actions.md`, `CHANGELOG.md`

- [ ] **Step 1: Fix the inverted versioning claim**

`kinova_gen3.repos` currently says, of `rammp-interfaces-ros2`:

> adding a field to an existing message is a MAJOR bump there, because Humble matches publishers to subscribers on type NAME and never checks content

The mechanism is right and the conclusion is backwards. That same mechanism is *why* appending is safe: the deserialiser stops at the end of the payload and leaves an absent trailing field at its declared default. Per that repo's own policy, a field **appended at the end with a default** is a **MINOR** bump; a field added anywhere else, or appended without a default, is MAJOR. Correct the comment and keep the warning about insertion, which is the genuinely dangerous case.

- [ ] **Step 2: Document the three new fields**

In `docs/guide-goto-ee-pose.md` and `docs/guide-goto-actions.md`: what each field does, that all three default to off, and — the part a caller will otherwise meet as a surprise — that **an approach also holds the other five pose components**, so the start must already match the goal on them. State that joint-space goals carry speed only.

Remove or correct any sentence saying there is no speed control; those pages currently say the planned trajectory always runs at full planner speed.

- [ ] **Step 3: CHANGELOG entry**

Name the three fields, the two new arm messages, the minor bump, and that out-of-range speed is refused with a reason rather than clamped.

- [ ] **Step 4: Full build and test, then commit**

---

### Task 6: Prove it end to end against the real planner

Everything above is unit-tested against a fake planner. This task runs the node in sim against the **real cuRobo container** on abra and drives it with a client.

- [ ] **Step 1: Start the planner and the node in sim on abra**

Use the sim image and the planner image already on the host. Do **not** start the arm driver, do not run any `sheppy` command, and do not command the physical arm — sim only.

- [ ] **Step 2: Send three `GoToEEPose` goals** — one plain, one with a lock, one with an approach offset — and record the resulting joint trajectories.

- [ ] **Step 3: Assert the differences are real**

The locked goal's trajectory must hold the locked components (check by comparing the planner's returned trajectory endpoints); the approach goal's path must differ measurably from the plain one; the scaled goal must take proportionally longer. A difference of zero means a field was dropped somewhere in the chain — which is exactly what this task exists to catch.

- [ ] **Step 4: Record the outcome** in the report, including the exact commands, so the next person can re-run it.

## Notes for the reviewer

- **The arm's message types are deliberately not the planner's**, and deliberately not named the same. If a diff makes `rammp_curobo_interfaces` appear outside `CuroboPlanClient`, that is a finding.
- **Every new field is appended with a default.** If any existing test needed editing to accommodate these changes, something is wrong: a client that sets nothing must be bit-for-bit unaffected.
- **The via-point-implies-holding semantic came from hardware**, not from reading docs. It is the thing most likely to confuse a caller, and the message comments are where they will look.
