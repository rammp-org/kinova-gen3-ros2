# GoToEEPose (cuRobo-backed) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `GoToEEPose` ROS2 action that delegates collision-free planning to the external cuRobo node and executes the returned joint trajectory through the existing `Supervisor` `CommandSink` seam.

**Architecture:** One new rclcpp action server (`GoToEEPoseServer`) hosted in the existing `kinova_arm_node`, backed by an async cuRobo action client (`CuroboPlanClient` → `/rammp_curobo/plan_to_pose`). A tiny `GoalRouter` fans the Supervisor's single `ActionServerPort` out by `GoalId` so the pre-existing `ExecuteJointTrajectory` backend and the new server share one Supervisor. Planning happens off the RT path entirely; the planned trajectory feeds the same lock-free Supervisor inbox `Ros2Backend` already uses.

**Tech Stack:** C++17, ROS2 Humble (rclcpp / rclcpp_action), ament_cmake, gtest (ament_add_gtest), Pinocchio (transitively, via core), the `kinova_lowlevel` core export.

**Design spec:** `docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md`.

## Global Constraints

- **Builds are aarch64-only on `abra`; muk cannot build.** Every build/test runs on abra via `scripts/abra_colcon.sh` (rsync muk→abra + colcon). See "Build & test loop" below.
- **ssh to abra must wrap in bash:** `ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; ..."'` — abra's default shell is zsh and chokes on ROS's `setup.bash`.
- **SI / radians internally.** No unit conversion in this layer (cuRobo already speaks rad/metres).
- **RT contract is untouched.** This round adds NO code to `compute` or the executor cycle. The planned trajectory reaches the RT loop only through the Supervisor's existing lock-free inbox. Do not touch the core beyond the single enum value in Task 2.
- **All new rclcpp code lives in ROS-adapter units** (`GoToEEPoseServer`, `CuroboPlanClient`); `GoalRouter` is rclcpp-free. The core (`kinova_lowlevel`) stays ROS-free.
- **Fail loud.** Reject a `GoToEEPose` goal whose `frame_id != base_link`; reject a planned trajectory the Supervisor won't accept.
- **Two repos, two branches.**
  - ROS2 work: `rammp-org/kinova_arm_ros2`, branch **`feat/goto-ee-pose-curobo`** (already created; the design spec is committed there).
  - Core one-line change: `rammp-org/kinova-gen3-driver`, branch **`feat/planning-failed-result-code`** (create in Task 2).
- **Commits:** conventional-commit subject; end every commit message with the two trailers:
  ```
  Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01QoSEXbPQLpXMQAfw6ntqJv
  ```
- **`kNumJoints == 7`**, `joint_1..joint_7` ordering, tool link `tool_frame` (baked into cuRobo config — not our concern).

## Build & test loop (abra) — referenced by every task's verify steps

The core working tree is rsynced live, so **local core edits are picked up without a core commit/push** (the commit in Task 2 is for provenance only).

**Build (sim, with tests), scoped so the cuRobo repo's GPU packages are never built:**
```sh
bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON
```

**Run one gtest binary on abra** (source both ROS and the workspace overlay so message typesupport resolves):
```sh
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; /tmp/kinova-ros2-ws/build/kinova_arm_ros2/<TESTBIN> --gtest_color=yes"'
```

**Build just the core** (Task 2 verification):
```sh
bash scripts/abra_colcon.sh --packages-select kinova_lowlevel
```

**Interfaces introspection** (Task 1 verification):
```sh
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; ros2 interface show kinova_arm_interfaces/action/GoToEEPose"'
```

---

### Task 1: `GoToEEPose.action` interface + `geometry_msgs` dependency

**Files:**
- Create: `kinova_arm_interfaces/action/GoToEEPose.action`
- Modify: `kinova_arm_interfaces/CMakeLists.txt`
- Modify: `kinova_arm_interfaces/package.xml`

**Interfaces:**
- Produces: `kinova_arm_interfaces/action/GoToEEPose` with `Goal{ geometry_msgs/PoseStamped target, string sender_id }`, `Result{ int32 error_code, string error_string, trajectory_msgs/JointTrajectoryPoint final_error }`, `Feedback{ string phase, string planner_state, float32 fraction_complete, trajectory_msgs/JointTrajectoryPoint actual }`.

- [ ] **Step 1: Create the action file**

`kinova_arm_interfaces/action/GoToEEPose.action`:
```
# Goal — move the tool (tool_frame) to a base_link pose; cuRobo plans collision-free.
geometry_msgs/PoseStamped target   # frame_id MUST be base_link (rejected otherwise)
string sender_id                   # arbitration hook, mirrors ExecuteJointTrajectory
---
# Result
int32   error_code   # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4,
                     # PREEMPTED=-6, PLANNING_FAILED=-7
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error   # joint-space final error
---
# Feedback
string  phase              # "planning" | "executing"
string  planner_state      # cuRobo feedback 'state', relayed while planning
float32 fraction_complete  # execution progress (0 while planning)
trajectory_msgs/JointTrajectoryPoint actual   # live measured q during execution
```

- [ ] **Step 2: Register the action + add `geometry_msgs` in CMakeLists**

In `kinova_arm_interfaces/CMakeLists.txt`, add `find_package(geometry_msgs REQUIRED)` after the other `find_package` lines, and update the generate call:
```cmake
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointTrajectory.action"
  "action/GoToEEPose.action"
  "msg/JointImpedanceGains.msg"
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs control_msgs action_msgs geometry_msgs)
```

- [ ] **Step 3: Add the `geometry_msgs` depend in package.xml**

In `kinova_arm_interfaces/package.xml`, add alongside the other `<depend>` lines:
```xml
  <depend>geometry_msgs</depend>
```

- [ ] **Step 4: Build and verify the interface generates**

Run (from "Build & test loop"): `bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON`
Then the interfaces-introspection command.
Expected: build green; `ros2 interface show kinova_arm_interfaces/action/GoToEEPose` prints the three sections with the fields above.

- [ ] **Step 5: Commit** (branch `feat/goto-ee-pose-curobo`)
```sh
git add kinova_arm_interfaces/action/GoToEEPose.action kinova_arm_interfaces/CMakeLists.txt kinova_arm_interfaces/package.xml
git commit   # feat(interfaces): add GoToEEPose.action (+ geometry_msgs dep)
```

---

### Task 2: Core `result_code::kPlanningFailed = -7`

**Files:**
- Modify: `../kinova-gen3-driver/include/kinova_lowlevel/interface/value_types.h:37-40` (core repo)

**Interfaces:**
- Produces: `kinova::interface::result_code::kPlanningFailed == -7`.

Note: this is the **core repo** (`/home/swapnil/atdev/kinova-gen3-driver`), branch `feat/planning-failed-result-code`. `-7` is the next free *custom* slot after this codebase's own `kPreempted = -6`; `-2`/`-3` are avoided because the other codes mirror `control_msgs/FollowJointTrajectory`, whose `-2`/`-3` mean `INVALID_JOINTS`/`OLD_HEADER_TIMESTAMP`.

- [ ] **Step 1: Create the core branch**
```sh
cd /home/swapnil/atdev/kinova-gen3-driver && git checkout -b feat/planning-failed-result-code
```

- [ ] **Step 2: Add the constant**

Replace the `result_code` namespace body in `include/kinova_lowlevel/interface/value_types.h`:
```cpp
namespace result_code {
  constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
                kGoalToleranceViolated = -5, kPreempted = -6, kPlanningFailed = -7;
}
```

- [ ] **Step 3: Build the core to confirm it compiles**

Run: `bash scripts/abra_colcon.sh --packages-select kinova_lowlevel` (from the ROS2 repo — the script rsyncs both trees).
Expected: `kinova_lowlevel` builds green. (Header-only constant; no runtime test needed — it is exercised by Task 4's mapping test.)

- [ ] **Step 4: Commit** (core repo, branch `feat/planning-failed-result-code`)
```sh
cd /home/swapnil/atdev/kinova-gen3-driver
git add include/kinova_lowlevel/interface/value_types.h
git commit   # feat(interface): add result_code::kPlanningFailed = -7 for the cuRobo tier
```

---

### Task 3: `GoalRouter` (ActionServerPort demux)

**Files:**
- Create: `kinova_arm_ros2/include/kinova_arm_ros2/goal_router.h`
- Create: `kinova_arm_ros2/src/goal_router.cpp`
- Create: `kinova_arm_ros2/test/goal_router_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt`

**Interfaces:**
- Consumes: `kinova::interface::ActionServerPort`, `GoalId`, `TrajectoryFeedback`, `TrajectoryResult` (core `ports.h`/`value_types.h`).
- Produces: `kinova_arm_ros2::GoalRouter` — `GoalRouter(ActionServerPort& default_port)`, `void register_owner(const GoalId&, ActionServerPort&)`, and the two `ActionServerPort` overrides. Unregistered ids route to `default_port`; `settle` clears the override.

- [ ] **Step 1: Write the failing test**

`kinova_arm_ros2/test/goal_router_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include "kinova_arm_ros2/goal_router.h"
using namespace kinova::interface;
namespace {
struct FakePort : ActionServerPort {
  int fb = 0, settled = 0;
  void publish_feedback(const GoalId&, const TrajectoryFeedback&) override { ++fb; }
  void settle(const GoalId&, const TrajectoryResult&) override { ++settled; }
};
GoalId mkid(uint8_t x) { GoalId id{}; id[0] = x; return id; }
}  // namespace

TEST(GoalRouter, UnregisteredFallsThroughToDefault) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.publish_feedback(mkid(1), {});
  r.settle(mkid(1), {});
  EXPECT_EQ(def.fb, 1);
  EXPECT_EQ(def.settled, 1);
  EXPECT_EQ(overlay.fb, 0);
  EXPECT_EQ(overlay.settled, 0);
}

TEST(GoalRouter, RegisteredOwnerReceivesFeedbackAndSettle) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.register_owner(mkid(2), overlay);
  r.publish_feedback(mkid(2), {});
  r.settle(mkid(2), {});
  EXPECT_EQ(overlay.fb, 1);
  EXPECT_EQ(overlay.settled, 1);
  EXPECT_EQ(def.fb, 0);
  EXPECT_EQ(def.settled, 0);
}

TEST(GoalRouter, SettleClearsOwnerSoNextRoutesToDefault) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.register_owner(mkid(3), overlay);
  r.settle(mkid(3), {});             // clears the override
  r.publish_feedback(mkid(3), {});   // now falls through to default
  EXPECT_EQ(overlay.settled, 1);
  EXPECT_EQ(def.fb, 1);
}
```

- [ ] **Step 2: Add the header (so the test compiles)**

`kinova_arm_ros2/include/kinova_arm_ros2/goal_router.h`:
```cpp
#pragma once
#include <map>
#include <mutex>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

// ActionServerPort demux. The Supervisor holds ONE ActionServerPort; this fans
// feedback/settle out by GoalId. Unregistered ids fall through to a default port
// (the pre-existing ExecuteJointTrajectory backend); overlay owners (the GoToEEPose
// server) register their ids explicitly. rclcpp-free.
class GoalRouter : public kinova::interface::ActionServerPort {
 public:
  explicit GoalRouter(kinova::interface::ActionServerPort& default_port);
  void register_owner(const kinova::interface::GoalId& id,
                      kinova::interface::ActionServerPort& owner);
  void publish_feedback(const kinova::interface::GoalId& id,
                        const kinova::interface::TrajectoryFeedback& fb) override;
  void settle(const kinova::interface::GoalId& id,
              const kinova::interface::TrajectoryResult& r) override;

 private:
  kinova::interface::ActionServerPort& default_;
  std::mutex m_;
  std::map<kinova::interface::GoalId, kinova::interface::ActionServerPort*> owners_;
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 3: Wire the test target in CMakeLists, build, verify it FAILS to link**

In `kinova_arm_ros2/CMakeLists.txt`, add the library (after the `message_mapping` block) and the test (inside the existing `if(BUILD_TESTING)` block):
```cmake
add_library(goal_router src/goal_router.cpp)
target_include_directories(goal_router PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(goal_router kinova_lowlevel::kinova_lowlevel)
```
```cmake
  ament_add_gtest(goal_router_test test/goal_router_test.cpp)
  target_link_libraries(goal_router_test goal_router)
```
Run the build. Expected: **link error** (undefined `GoalRouter::…`) because `src/goal_router.cpp` does not exist yet — confirms the test is wired and needs the implementation.

- [ ] **Step 4: Implement `goal_router.cpp`**

`kinova_arm_ros2/src/goal_router.cpp`:
```cpp
#include "kinova_arm_ros2/goal_router.h"
namespace kinova_arm_ros2 {
using namespace kinova::interface;

GoalRouter::GoalRouter(ActionServerPort& default_port) : default_(default_port) {}

void GoalRouter::register_owner(const GoalId& id, ActionServerPort& owner) {
  std::lock_guard<std::mutex> l(m_);
  owners_[id] = &owner;
}

void GoalRouter::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  ActionServerPort* p;
  { std::lock_guard<std::mutex> l(m_);
    auto it = owners_.find(id);
    p = (it == owners_.end()) ? &default_ : it->second; }
  p->publish_feedback(id, fb);
}

void GoalRouter::settle(const GoalId& id, const TrajectoryResult& r) {
  ActionServerPort* p;
  { std::lock_guard<std::mutex> l(m_);
    auto it = owners_.find(id);
    if (it == owners_.end()) { p = &default_; }
    else { p = it->second; owners_.erase(it); } }   // clear override before settling
  p->settle(id, r);
}
}  // namespace kinova_arm_ros2
```

- [ ] **Step 5: Build + run the test, verify PASS**

Build, then run test binary `goal_router_test` (see "Build & test loop").
Expected: 3/3 PASS.

- [ ] **Step 6: Commit**
```sh
git add kinova_arm_ros2/include/kinova_arm_ros2/goal_router.h kinova_arm_ros2/src/goal_router.cpp kinova_arm_ros2/test/goal_router_test.cpp kinova_arm_ros2/CMakeLists.txt
git commit   # feat(ros2): GoalRouter — ActionServerPort demux by GoalId
```

---

### Task 4: message-mapping additions (JointTrajectory→TrajectoryGoal + GoToEEPose msgs)

**Files:**
- Modify: `kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h`
- Modify: `kinova_arm_ros2/src/message_mapping.cpp`
- Modify: `kinova_arm_ros2/test/message_mapping_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt` (add `trajectory_msgs` dep on `message_mapping`)

**Interfaces:**
- Consumes: Task 1 (`GoToEEPose`), Task 2 (`kPlanningFailed`), core `TrajectoryGoal`/`TrajectoryFeedback`/`TrajectoryResult`.
- Produces:
  - `kinova::interface::TrajectoryGoal to_trajectory_goal(const trajectory_msgs::msg::JointTrajectory& traj)` — position-mode, `kLatestWins`, waypoints from `positions` + `time_from_start`; **caller sets `path_tolerance`/`sender_id`**.
  - `GoToEEPose::Feedback to_goto_feedback_msg(const kinova::interface::TrajectoryFeedback& fb)` — `phase="executing"`, `fraction_complete`, `actual`.
  - `GoToEEPose::Result to_goto_result_msg(const kinova::interface::TrajectoryResult& r)`.

- [ ] **Step 1: Write the failing tests**

Append to `kinova_arm_ros2/test/message_mapping_test.cpp` (the `pt(v,t)` helper already exists in this file):
```cpp
TEST(MessageMapping, JointTrajectoryToPositionGoal) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.points = { pt(0.0, 0.0), pt(0.3, 0.5) };
  auto tg = to_trajectory_goal(traj);
  ASSERT_EQ(tg.trajectory.points.size(), 2u);
  EXPECT_NEAR(tg.trajectory.points[1].q[0], 0.3, 1e-12);
  EXPECT_NEAR(tg.trajectory.points[1].t_s, 0.5, 1e-9);
  EXPECT_EQ(tg.control_mode, ControlModeKind::kPosition);
  EXPECT_EQ(tg.preemption, Preemption::kLatestWins);
  EXPECT_FALSE(tg.has_gains);
}

TEST(MessageMapping, GotoResultCarriesPlanningFailed) {
  kinova::interface::TrajectoryResult r;
  r.error_code = kinova::interface::result_code::kPlanningFailed;
  r.error_string = "no plan";
  r.final_error = kinova::JointVec::Zero();
  auto m = to_goto_result_msg(r);
  EXPECT_EQ(m.error_code, -7);
  EXPECT_EQ(m.error_string, "no plan");
  ASSERT_EQ(m.final_error.positions.size(), 7u);
}

TEST(MessageMapping, GotoExecutingFeedback) {
  kinova::interface::TrajectoryFeedback fb;
  fb.fraction_complete = 0.5;
  fb.actual = kinova::JointVec::Constant(0.2);
  auto m = to_goto_feedback_msg(fb);
  EXPECT_EQ(m.phase, "executing");
  EXPECT_NEAR(m.fraction_complete, 0.5f, 1e-6);
  ASSERT_EQ(m.actual.positions.size(), 7u);
  EXPECT_NEAR(m.actual.positions[0], 0.2, 1e-12);
}
```

- [ ] **Step 2: Declare the new functions in the header**

In `kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h`, add includes + declarations:
```cpp
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
```
```cpp
using GoToEEPose = kinova_arm_interfaces::action::GoToEEPose;

kinova::interface::TrajectoryGoal to_trajectory_goal(
    const trajectory_msgs::msg::JointTrajectory& traj);
GoToEEPose::Feedback to_goto_feedback_msg(const kinova::interface::TrajectoryFeedback& fb);
GoToEEPose::Result   to_goto_result_msg(const kinova::interface::TrajectoryResult& r);
```

- [ ] **Step 3: Run the tests, verify they FAIL (undefined reference)**

Add `trajectory_msgs` to the `message_mapping` deps in `CMakeLists.txt`:
```cmake
ament_target_dependencies(message_mapping rclcpp kinova_arm_interfaces trajectory_msgs)
```
Build + run `message_mapping_test`. Expected: link/compile failure on the three new symbols.

- [ ] **Step 4: Implement the mappers**

Append to `kinova_arm_ros2/src/message_mapping.cpp` (the file-static `vec_to_point` is already defined above these):
```cpp
TrajectoryGoal to_trajectory_goal(const trajectory_msgs::msg::JointTrajectory& traj) {
  TrajectoryGoal tg;
  for (const auto& p : traj.points) {
    JointWaypoint w{JointVec::Zero(), 0.0};
    for (int i = 0; i < kNumJoints && i < static_cast<int>(p.positions.size()); ++i)
      w.q[i] = p.positions[i];
    w.t_s = static_cast<double>(p.time_from_start.sec)
          + static_cast<double>(p.time_from_start.nanosec) * 1e-9;
    tg.trajectory.points.push_back(w);
  }
  tg.control_mode = ControlModeKind::kPosition;
  tg.preemption   = Preemption::kLatestWins;
  // path_tolerance / sender_id are set by the caller (GoToEEPoseServer).
  return tg;
}

GoToEEPose::Feedback to_goto_feedback_msg(const TrajectoryFeedback& fb) {
  GoToEEPose::Feedback m;
  m.phase = "executing";
  m.fraction_complete = static_cast<float>(fb.fraction_complete);
  m.actual = vec_to_point(fb.actual);
  return m;
}

GoToEEPose::Result to_goto_result_msg(const TrajectoryResult& r) {
  GoToEEPose::Result m;
  m.error_code = r.error_code;
  m.error_string = r.error_string;
  m.final_error = vec_to_point(r.final_error);
  return m;
}
```

- [ ] **Step 5: Build + run `message_mapping_test`, verify PASS**

Expected: all tests (the 5 pre-existing + 3 new) PASS.

- [ ] **Step 6: Commit**
```sh
git add kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h kinova_arm_ros2/src/message_mapping.cpp kinova_arm_ros2/test/message_mapping_test.cpp kinova_arm_ros2/CMakeLists.txt
git commit   # feat(ros2): map cuRobo JointTrajectory + GoToEEPose feedback/result
```

---

### Task 5: `CuroboPlanClient` (async plan_to_pose client) + cuRobo interfaces in the build

**Files:**
- Create: `kinova_arm_ros2/include/kinova_arm_ros2/curobo_plan_client.h`
- Create: `kinova_arm_ros2/src/curobo_plan_client.cpp`
- Create: `kinova_arm_ros2/test/fake_curobo_server.h` (shared test fake; reused in Task 6)
- Create: `kinova_arm_ros2/test/curobo_plan_client_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt`, `kinova_arm_ros2/package.xml`, `kinova_arm.repos`, `scripts/abra_colcon.sh`

**Interfaces:**
- Consumes: `rammp_curobo_interfaces/action/PlanToPose`.
- Produces: `kinova_arm_ros2::CuroboPlanClient` with `struct Outcome{ bool ok; std::string message; trajectory_msgs::msg::JointTrajectory trajectory; }`, `CuroboPlanClient(rclcpp::Node::SharedPtr, rclcpp::CallbackGroup::SharedPtr, std::string action_name="/rammp_curobo/plan_to_pose")`, `void plan(const geometry_msgs::msg::Pose&, FeedbackCb, DoneCb)` (calls `DoneCb` exactly once), `void cancel()`. `FeedbackCb = std::function<void(const std::string&)>`, `DoneCb = std::function<void(Outcome)>`.
- Produces (test): `kinova_arm_ros2::test::FakeCuroboServer(node, bool succeed, int n_points=3)`.

- [ ] **Step 1: Make `rammp_curobo_interfaces` available to the build**

The abra dev loop rsyncs local trees (it does not vcs-import `.repos`). Add the interfaces package to the workspace by rsyncing just that sub-package (its GPU siblings are never copied, so colcon never sees them).

Clone the cuRobo repo locally once (stable path, so the rsync source persists):
```sh
git clone --depth 1 https://github.com/ChrissCox/RAMMP-CuRobo /home/swapnil/atdev/RAMMP-CuRobo
```
In `scripts/abra_colcon.sh`, add a third rsync (after the existing two), copying **only** the interfaces package:
```sh
rsync -az --mkpath --delete --exclude '.git/' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "/home/swapnil/atdev/RAMMP-CuRobo/rammp_curobo_interfaces/" "abra:$WS/src/rammp_curobo_interfaces/"
```
Document the source in `kinova_arm.repos` (for the container build) by appending:
```yaml
  RAMMP-CuRobo:
    type: git
    url: https://github.com/ChrissCox/RAMMP-CuRobo.git
    version: main
    # Only rammp_curobo_interfaces is a build dep (dependency-free IDL, no GPU stack).
    # The bare-metal loop rsyncs just that sub-package; container builds must scope
    # colcon with --packages-up-to kinova_arm_ros2 so the GPU packages are skipped.
```

- [ ] **Step 2: Write the shared fake cuRobo server**

`kinova_arm_ros2/test/fake_curobo_server.h`:
```cpp
#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2::test {

// Minimal fake /rammp_curobo/plan_to_pose server. succeed=true returns a canned
// n-point joint_1..7 trajectory; succeed=false aborts with a message.
class FakeCuroboServer {
 public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanToPose>;

  FakeCuroboServer(rclcpp::Node::SharedPtr node, bool succeed, int n_points = 3)
      : node_(node), succeed_(succeed), n_points_(n_points) {
    server_ = rclcpp_action::create_server<PlanToPose>(
        node_, "/rammp_curobo/plan_to_pose",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanToPose::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<GoalHandle> gh) { execute(gh); });
  }

 private:
  void execute(std::shared_ptr<GoalHandle> gh) {
    auto result = std::make_shared<PlanToPose::Result>();
    if (!succeed_) {
      result->success = false;
      result->message = "fake planner: no solution";
      gh->abort(result);
      return;
    }
    result->success = true;
    result->message = "fake plan ok";
    result->trajectory.joint_names =
        {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
    for (int k = 0; k < n_points_; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(7, 0.01 * (k + 1));
      const double t = 0.02 * (k + 1);
      p.time_from_start.sec = static_cast<int32_t>(t);
      p.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
      result->trajectory.points.push_back(p);
    }
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<PlanToPose>::SharedPtr server_;
  bool succeed_;
  int n_points_;
};
}  // namespace kinova_arm_ros2::test
```

- [ ] **Step 3: Write the failing client test**

`kinova_arm_ros2/test/curobo_plan_client_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "fake_curobo_server.h"
using namespace std::chrono_literals;
using kinova_arm_ros2::CuroboPlanClient;

class CuroboClientTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(CuroboClientTest, PlanSuccessReturnsTrajectory) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.trajectory.points.size(), 3u);

  ex.cancel();
  spin.join();
}

TEST_F(CuroboClientTest, PlanAbortReturnsFailure) {
  auto node = std::make_shared<rclcpp::Node>("curobo_client_test2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  CuroboPlanClient client(node, grp);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  std::promise<CuroboPlanClient::Outcome> p;
  auto f = p.get_future();
  client.plan(geometry_msgs::msg::Pose{}, nullptr,
              [&](CuroboPlanClient::Outcome o) { p.set_value(std::move(o)); });
  ASSERT_EQ(f.wait_for(5s), std::future_status::ready);
  auto o = f.get();
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.message.empty());

  ex.cancel();
  spin.join();
}
```

- [ ] **Step 4: Add the client header**

`kinova_arm_ros2/include/kinova_arm_ros2/curobo_plan_client.h`:
```cpp
#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2 {

// Async client to the external cuRobo planner (/rammp_curobo/plan_to_pose).
// The ONLY unit that knows cuRobo exists. plan() dispatches and returns; the
// result arrives on on_done from the rclcpp executor (client's reentrant group).
// on_done is invoked EXACTLY ONCE (success, failure, rejection, or unavailable).
class CuroboPlanClient {
 public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  struct Outcome {
    bool ok = false;
    std::string message;
    trajectory_msgs::msg::JointTrajectory trajectory;
  };
  using FeedbackCb = std::function<void(const std::string& state)>;
  using DoneCb = std::function<void(Outcome)>;

  CuroboPlanClient(rclcpp::Node::SharedPtr node,
                   rclcpp::CallbackGroup::SharedPtr cb_group,
                   std::string action_name = "/rammp_curobo/plan_to_pose");
  void plan(const geometry_msgs::msg::Pose& target, FeedbackCb on_fb, DoneCb on_done);
  void cancel();

 private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlanToPose>;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanToPose>::SharedPtr client_;
  std::mutex m_;
  std::shared_ptr<GoalHandle> active_;   // in-flight goal, for cancel()
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 5: Wire libs/deps in CMakeLists + package.xml; build; verify test FAILS to link**

`kinova_arm_ros2/CMakeLists.txt` — add `find_package`s near the top:
```cmake
find_package(geometry_msgs REQUIRED)
find_package(rammp_curobo_interfaces REQUIRED)
```
Add the library (after `goal_router`):
```cmake
add_library(curobo_plan_client src/curobo_plan_client.cpp)
target_include_directories(curobo_plan_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(curobo_plan_client
  rclcpp rclcpp_action rammp_curobo_interfaces geometry_msgs trajectory_msgs)
```
Add the test (inside `if(BUILD_TESTING)`):
```cmake
  ament_add_gtest(curobo_plan_client_test test/curobo_plan_client_test.cpp)
  target_include_directories(curobo_plan_client_test PRIVATE test)
  target_link_libraries(curobo_plan_client_test curobo_plan_client)
  ament_target_dependencies(curobo_plan_client_test
    rclcpp rclcpp_action rammp_curobo_interfaces geometry_msgs)
```
`kinova_arm_ros2/package.xml` — add:
```xml
  <depend>geometry_msgs</depend>
  <depend>rammp_curobo_interfaces</depend>
```
Build. Expected: `curobo_plan_client_test` fails to link (`CuroboPlanClient::…` undefined) — confirms wiring.

- [ ] **Step 6: Implement `curobo_plan_client.cpp`**

`kinova_arm_ros2/src/curobo_plan_client.cpp`:
```cpp
#include "kinova_arm_ros2/curobo_plan_client.h"
#include <chrono>
#include <mutex>
namespace kinova_arm_ros2 {

CuroboPlanClient::CuroboPlanClient(rclcpp::Node::SharedPtr node,
                                   rclcpp::CallbackGroup::SharedPtr cb_group,
                                   std::string action_name)
    : node_(node) {
  client_ = rclcpp_action::create_client<PlanToPose>(node_, action_name, cb_group);
}

void CuroboPlanClient::plan(const geometry_msgs::msg::Pose& target,
                            FeedbackCb on_fb, DoneCb on_done) {
  // Guarantee on_done fires exactly once across the goal-response / result paths.
  auto once = std::make_shared<std::once_flag>();
  auto fire = [once, on_done](Outcome o) {
    std::call_once(*once, [&] { on_done(std::move(o)); });
  };

  if (!client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    fire({false, "cuRobo action server unavailable", {}});
    return;
  }

  PlanToPose::Goal goal;
  goal.target = target;
  goal.start_joints = start_joints;   // plan from where the arm actually is

  rclcpp_action::Client<PlanToPose>::SendGoalOptions opts;
  opts.goal_response_callback = [this, fire](std::shared_ptr<GoalHandle> gh) {
    if (!gh) { fire({false, "cuRobo rejected plan goal", {}}); return; }
    std::lock_guard<std::mutex> l(m_);
    active_ = gh;
  };
  opts.feedback_callback = [on_fb](std::shared_ptr<GoalHandle>,
                                   const std::shared_ptr<const PlanToPose::Feedback> fb) {
    if (on_fb) on_fb(fb->state);
  };
  opts.result_callback = [this, fire](const GoalHandle::WrappedResult& wr) {
    { std::lock_guard<std::mutex> l(m_); active_.reset(); }
    Outcome o;
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result && wr.result->success) {
      o.ok = true;
      o.message = wr.result->message;
      o.trajectory = wr.result->trajectory;
    } else {
      o.ok = false;
      o.message = (wr.result && !wr.result->message.empty()) ? wr.result->message
                                                             : "cuRobo plan failed";
    }
    fire(std::move(o));
  };
  client_->async_send_goal(goal, opts);
}

void CuroboPlanClient::cancel() {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); gh = active_; }
  if (gh) client_->async_cancel_goal(gh);
}
}  // namespace kinova_arm_ros2
```

- [ ] **Step 7: Build + run `curobo_plan_client_test`, verify PASS**

Expected: 2/2 PASS.

- [ ] **Step 8: Commit**
```sh
git add kinova_arm_ros2/include/kinova_arm_ros2/curobo_plan_client.h kinova_arm_ros2/src/curobo_plan_client.cpp kinova_arm_ros2/test/fake_curobo_server.h kinova_arm_ros2/test/curobo_plan_client_test.cpp kinova_arm_ros2/CMakeLists.txt kinova_arm_ros2/package.xml kinova_arm.repos scripts/abra_colcon.sh
git commit   # feat(ros2): CuroboPlanClient — async /rammp_curobo/plan_to_pose client
```

---

### Task 6: `GoToEEPoseServer` + end-to-end integration test

**Files:**
- Create: `kinova_arm_ros2/include/kinova_arm_ros2/goto_ee_pose_server.h`
- Create: `kinova_arm_ros2/src/goto_ee_pose_server.cpp`
- Create: `kinova_arm_ros2/test/goto_ee_pose_integration_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt`

**Interfaces:**
- Consumes: `GoalRouter` (T3), `to_trajectory_goal(JointTrajectory)`/`to_goto_*` (T4), `CuroboPlanClient` (T5), core `CommandSink`/`ActionServerPort`/`result_code`.
- Produces: `kinova_arm_ros2::GoToEEPoseServer(rclcpp::Node::SharedPtr, GoalRouter&, CuroboPlanClient&, rclcpp::CallbackGroup::SharedPtr)`, `void set_command_sink(kinova::interface::CommandSink*)`, and the two `ActionServerPort` overrides (execution-phase feedback/settle, driven via the router).

- [ ] **Step 1: Write the failing integration test**

`kinova_arm_ros2/test/goto_ee_pose_integration_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/goto_ee_pose_server.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_curobo_server.h"
using namespace std::chrono_literals;
using namespace kinova::interface;
using GoToEEPose = kinova_arm_interfaces::action::GoToEEPose;

namespace {
// Stand-in for the Supervisor: records the submitted goal and, on accept,
// immediately drives the router as if execution completed successfully.
struct FakeSupervisor : public CommandSink {
  ActionServerPort& port;
  bool accept = true;
  bool got_goal = false;
  TrajectoryGoal last_goal;
  explicit FakeSupervisor(ActionServerPort& p) : port(p) {}
  GoalResponse on_trajectory_goal(const TrajectoryGoal& g) override {
    last_goal = g; got_goal = true;
    return accept ? GoalResponse::kAccept : GoalResponse::kReject;
  }
  void on_trajectory_accepted(const GoalId& id, const TrajectoryGoal&) override {
    TrajectoryResult r; r.error_code = result_code::kSuccessful;
    r.final_error = JointVec::Zero();
    port.settle(id, r);   // simulate immediate successful execution
  }
  CancelResponse on_trajectory_cancel(const GoalId& id) override {
    TrajectoryResult r; r.error_code = result_code::kPreempted;
    port.settle(id, r); return CancelResponse::kAccept;
  }
  GainsResult on_set_gains(const GainsRequest&) override { return {}; }
  ArmState on_query_state() override { return {}; }
};
struct DummyPort : public ActionServerPort {   // default router port; unused here
  void publish_feedback(const GoalId&, const TrajectoryFeedback&) override {}
  void settle(const GoalId&, const TrajectoryResult&) override {}
};

// Send a GoToEEPose goal and block for its result code.
int send_and_get_code(rclcpp::Node::SharedPtr node, const std::string& frame) {
  auto client = rclcpp_action::create_client<GoToEEPose>(node, "go_to_ee_pose");
  if (!client->wait_for_action_server(5s)) return 999;
  GoToEEPose::Goal goal;
  goal.target.header.frame_id = frame;
  std::promise<int> code;
  auto fut = code.get_future();
  rclcpp_action::Client<GoToEEPose>::SendGoalOptions opts;
  opts.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<GoToEEPose>::WrappedResult& wr) {
        code.set_value(wr.result ? wr.result->error_code : -12345);
      };
  client->async_send_goal(goal, opts);
  if (fut.wait_for(8s) != std::future_status::ready) return 888;
  return fut.get();
}
}  // namespace

class GotoServerTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(GotoServerTest, PlanSuccessDrivesTrajectoryAndSucceeds) {
  auto node = std::make_shared<rclcpp::Node>("goto_it");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/true, /*n_points=*/3);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kSuccessful);
  EXPECT_TRUE(sup.got_goal);
  EXPECT_EQ(sup.last_goal.trajectory.points.size(), 3u);
  EXPECT_EQ(sup.last_goal.control_mode, ControlModeKind::kPosition);

  ex.cancel();
  spin.join();
}

TEST_F(GotoServerTest, PlanFailureSettlesPlanningFailed) {
  auto node = std::make_shared<rclcpp::Node>("goto_it2");
  kinova_arm_ros2::test::FakeCuroboServer fake(node, /*succeed=*/false);
  auto grp = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, grp);
  DummyPort dummy;
  kinova_arm_ros2::GoalRouter router(dummy);
  kinova_arm_ros2::GoToEEPoseServer server(node, router, planner, grp);
  FakeSupervisor sup(router);
  server.set_command_sink(&sup);

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spin([&] { ex.spin(); });

  const int code = send_and_get_code(node, "base_link");
  EXPECT_EQ(code, result_code::kPlanningFailed);
  EXPECT_FALSE(sup.got_goal);

  ex.cancel();
  spin.join();
}
```

- [ ] **Step 2: Add the server header**

`kinova_arm_ros2/include/kinova_arm_ros2/goto_ee_pose_server.h`:
```cpp
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

// Hosts GoToEEPose: validate -> cuRobo plan -> feed the planned trajectory into the
// shared CommandSink seam (same path as ExecuteJointTrajectory) -> settle. Implements
// ActionServerPort for its OWN goals; the GoalRouter routes the Supervisor's
// execution feedback/settle back here by GoalId.
class GoToEEPoseServer : public kinova::interface::ActionServerPort {
 public:
  using Action = kinova_arm_interfaces::action::GoToEEPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                   CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb_group);
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

  // ActionServerPort (called by the supervisor sampler thread via the router):
  void publish_feedback(const kinova::interface::GoalId&,
                        const kinova::interface::TrajectoryFeedback&) override;
  void settle(const kinova::interface::GoalId&,
              const kinova::interface::TrajectoryResult&) override;

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Action::Goal>);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
  void handle_accepted(std::shared_ptr<GoalHandle>);
  void on_plan_done(kinova::interface::GoalId id, CuroboPlanClient::Outcome outcome);
  void settle_local(std::shared_ptr<GoalHandle> gh, int error_code, const std::string& msg);

  static constexpr double kGotoPathTolRad = 0.35;  // generous; full-speed tracking lag

  rclcpp::Node::SharedPtr node_;
  GoalRouter& router_;
  CuroboPlanClient& planner_;
  kinova::interface::CommandSink* sink_ = nullptr;
  rclcpp_action::Server<Action>::SharedPtr server_;

  struct Goal { std::shared_ptr<GoalHandle> gh; bool executing = false; };
  std::mutex m_;
  std::map<kinova::interface::GoalId, Goal> goals_;
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 3: Wire libs in CMakeLists; build; verify integration test FAILS to link**

`kinova_arm_ros2/CMakeLists.txt` — add the library (after `curobo_plan_client`):
```cmake
add_library(goto_ee_pose_server src/goto_ee_pose_server.cpp)
target_include_directories(goto_ee_pose_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(goto_ee_pose_server
  rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(goto_ee_pose_server
  goal_router curobo_plan_client message_mapping kinova_lowlevel::kinova_lowlevel)
```
Add the test (inside `if(BUILD_TESTING)`):
```cmake
  ament_add_gtest(goto_ee_pose_integration_test test/goto_ee_pose_integration_test.cpp)
  target_include_directories(goto_ee_pose_integration_test PRIVATE test)
  target_link_libraries(goto_ee_pose_integration_test
    goto_ee_pose_server goal_router curobo_plan_client message_mapping)
  ament_target_dependencies(goto_ee_pose_integration_test
    rclcpp rclcpp_action kinova_arm_interfaces rammp_curobo_interfaces geometry_msgs)
```
Build. Expected: `goto_ee_pose_integration_test` fails to link (`GoToEEPoseServer::…` undefined).

- [ ] **Step 4: Implement `goto_ee_pose_server.cpp`**

`kinova_arm_ros2/src/goto_ee_pose_server.cpp`:
```cpp
#include "kinova_arm_ros2/goto_ee_pose_server.h"
#include <functional>
#include "kinova_arm_ros2/message_mapping.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_arm_ros2 {
using namespace kinova::interface;
using std::placeholders::_1;
using std::placeholders::_2;

GoToEEPoseServer::GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                                   CuroboPlanClient& planner,
                                   rclcpp::CallbackGroup::SharedPtr cb_group)
    : node_(node), router_(router), planner_(planner) {
  server_ = rclcpp_action::create_server<Action>(
      node_, "go_to_ee_pose",
      std::bind(&GoToEEPoseServer::handle_goal, this, _1, _2),
      std::bind(&GoToEEPoseServer::handle_cancel, this, _1),
      std::bind(&GoToEEPoseServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(), cb_group);
}

rclcpp_action::GoalResponse GoToEEPoseServer::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
  if (!sink_) return rclcpp_action::GoalResponse::REJECT;
  if (goal->target.header.frame_id != "base_link") {   // fail loud
    RCLCPP_WARN(node_->get_logger(),
                "rejecting GoToEEPose: frame_id '%s' != base_link",
                goal->target.header.frame_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToEEPoseServer::handle_cancel(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  bool executing = false;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it != goals_.end()) executing = it->second.executing; }
  if (executing) {
    if (sink_) sink_->on_trajectory_cancel(id);   // Supervisor -> kPreempted -> settle()
  } else {
    planner_.cancel();                            // cancel in-flight plan -> on_plan_done
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToEEPoseServer::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  { std::lock_guard<std::mutex> l(m_); goals_[id] = Goal{gh, false}; }
  auto planning = std::make_shared<Action::Feedback>();
  planning->phase = "planning";
  gh->publish_feedback(planning);

  const geometry_msgs::msg::Pose target = gh->get_goal()->target.pose;
  planner_.plan(
      target,
      [this, id](const std::string& state) {
        std::shared_ptr<GoalHandle> gh2;
        { std::lock_guard<std::mutex> l(m_);
          auto it = goals_.find(id);
          if (it == goals_.end()) return;
          gh2 = it->second.gh; }
        auto f = std::make_shared<Action::Feedback>();
        f->phase = "planning";
        f->planner_state = state;
        gh2->publish_feedback(f);
      },
      [this, id](CuroboPlanClient::Outcome o) { on_plan_done(id, std::move(o)); });
}

void GoToEEPoseServer::on_plan_done(GoalId id, CuroboPlanClient::Outcome outcome) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh; }

  if (!outcome.ok) {
    const bool canceled = gh->is_canceling();
    settle_local(gh, canceled ? result_code::kPreempted : result_code::kPlanningFailed,
                 canceled ? "canceled during planning" : outcome.message);
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }

  TrajectoryGoal tg = to_trajectory_goal(outcome.trajectory);   // position mode
  tg.path_tolerance = kinova::JointVec::Constant(kGotoPathTolRad);
  tg.sender_id = gh->get_goal()->sender_id;

  const GoalResponse r = sink_->on_trajectory_goal(tg);
  if (r != GoalResponse::kAccept) {
    settle_local(gh, result_code::kInvalidGoal, "supervisor rejected planned trajectory");
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it != goals_.end()) it->second.executing = true; }
  router_.register_owner(id, *this);
  sink_->on_trajectory_accepted(id, tg);
}

void GoToEEPoseServer::settle_local(std::shared_ptr<GoalHandle> gh, int error_code,
                                    const std::string& msg) {
  TrajectoryResult r;
  r.error_code = error_code;
  r.error_string = msg;
  r.final_error = kinova::JointVec::Zero();
  auto out = std::make_shared<Action::Result>(to_goto_result_msg(r));
  if (gh->is_canceling())                          gh->canceled(out);
  else if (error_code == result_code::kSuccessful) gh->succeed(out);
  else                                             gh->abort(out);
}

// --- ActionServerPort (execution phase, sampler thread via router) ---
void GoToEEPoseServer::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh; }
  auto msg = std::make_shared<Action::Feedback>(to_goto_feedback_msg(fb));
  gh->publish_feedback(msg);
}

void GoToEEPoseServer::settle(const GoalId& id, const TrajectoryResult& r) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh;
    goals_.erase(it); }
  auto msg = std::make_shared<Action::Result>(to_goto_result_msg(r));
  if (gh->is_canceling())                            gh->canceled(msg);
  else if (r.error_code == result_code::kSuccessful) gh->succeed(msg);
  else                                               gh->abort(msg);
}
}  // namespace kinova_arm_ros2
```

- [ ] **Step 5: Build + run `goto_ee_pose_integration_test`, verify PASS**

Expected: 2/2 PASS — success path settles `SUCCESSFUL` with a 3-point position goal submitted; failure path settles `PLANNING_FAILED` with no submission.

- [ ] **Step 6: Commit**
```sh
git add kinova_arm_ros2/include/kinova_arm_ros2/goto_ee_pose_server.h kinova_arm_ros2/src/goto_ee_pose_server.cpp kinova_arm_ros2/test/goto_ee_pose_integration_test.cpp kinova_arm_ros2/CMakeLists.txt
git commit   # feat(ros2): GoToEEPoseServer — plan via cuRobo, execute via Supervisor
```

---

### Task 7: Bring-up wiring (MultiThreadedExecutor + the new server) + test client

**Files:**
- Modify: `kinova_arm_ros2/src/bringup_node.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt` (link new libs + `geometry_msgs` into the node)
- Create: `kinova_arm_ros2/test/send_goto_pose.py`

**Interfaces:**
- Consumes: `GoalRouter`, `CuroboPlanClient`, `GoToEEPoseServer`, the core `Supervisor` (whose `ActionServerPort` argument becomes the router).

- [ ] **Step 1: Rewire `bringup_node.cpp`**

Add includes near the existing backend include:
```cpp
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/goto_ee_pose_server.h"
```
Replace the node/backend/supervisor construction block (currently lines ~52-55) with:
```cpp
  auto node = std::make_shared<rclcpp::Node>("kinova_arm_node");
  auto backend = std::make_shared<kinova_arm_ros2::Ros2Backend>(node);

  // Router demuxes the Supervisor's single ActionServerPort by GoalId; the
  // pre-existing ExecuteJointTrajectory backend is the default (fall-through) port.
  kinova_arm_ros2::GoalRouter router(*backend);
  // Async cuRobo planning + the high-level server run on a reentrant group so the
  // plan round-trip never starves the ExecuteJointTrajectory server/feedback.
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, cb_group);
  kinova_arm_ros2::GoToEEPoseServer goto_server(node, router, planner, cb_group);

  interface::Supervisor sup(pos, imp, exec, snap, pump_dyn, *backend, router);
  backend->set_command_sink(&sup);
  goto_server.set_command_sink(&sup);
```
Change the ROS spin thread from single- to multi-threaded (currently line ~66):
```cpp
  std::thread ros_spin([&]{ rclcpp::executors::MultiThreadedExecutor ex; ex.add_node(node);
    while (!g_stop.load() && rclcpp::ok()) ex.spin_some(std::chrono::milliseconds(10)); });
```
Update the startup log line (~70):
```cpp
  RCLCPP_INFO(node->get_logger(),
              "kinova_arm_node up (%s); actions: /execute_joint_trajectory, /go_to_ee_pose",
              use_sim ? "sim" : "real");
```

- [ ] **Step 2: Link the new libs into the node**

In `kinova_arm_ros2/CMakeLists.txt`, update the node's deps/links:
```cmake
ament_target_dependencies(kinova_arm_node rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(kinova_arm_node
  ros2_backend goto_ee_pose_server goal_router curobo_plan_client message_mapping
  kinova_lowlevel::kinova_lowlevel)
```

- [ ] **Step 3: Build the node, verify it links**

Run: `bash scripts/abra_colcon.sh --packages-up-to kinova_arm_ros2 --cmake-args -DBUILD_TESTING=ON`
Expected: `kinova_arm_node` builds and installs green.

- [ ] **Step 4: Launch the node (sim) and verify both actions are advertised**
```sh
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver; (ros2 run kinova_arm_ros2 kinova_arm_node --sim --urdf models/gen3_7dof_2f85.urdf &) ; sleep 4; ros2 action list; pkill -TERM -f /tmp/kinova-ros2-ws/install/kinova_arm_ros2/lib/kinova_arm_ros2/kinova_arm_node; sleep 1"'
```
Expected: `ros2 action list` shows **both** `/execute_joint_trajectory` and `/go_to_ee_pose`. Confirm the node process is gone (shared abra — never leak a servoing node).

- [ ] **Step 5: Add the test client `send_goto_pose.py`**

`kinova_arm_ros2/test/send_goto_pose.py`:
```python
#!/usr/bin/env python3
"""Send a GoToEEPose goal (base_link target) and print feedback + result.

The operator supplies a base_link tool pose. Pick a pose near the current tool
pose for a safe local move; cuRobo plans collision-free from the live /joint_states.
"""
import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from kinova_arm_interfaces.action import GoToEEPose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                    help="target tool position in base_link (metres)")
    ap.add_argument("--quat", type=float, nargs=4, required=True,
                    metavar=("X", "Y", "Z", "W"), help="target tool orientation xyzw")
    ap.add_argument("--sender-id", default="send_goto_pose")
    args = ap.parse_args()

    rclpy.init()
    node = Node("send_goto_pose")
    client = ActionClient(node, GoToEEPose, "go_to_ee_pose")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("go_to_ee_pose action server not available")
        return 1

    goal = GoToEEPose.Goal()
    goal.target.header.frame_id = "base_link"
    goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = args.pos
    (goal.target.pose.orientation.x, goal.target.pose.orientation.y,
     goal.target.pose.orientation.z, goal.target.pose.orientation.w) = args.quat
    goal.sender_id = args.sender_id

    def on_fb(fb):
        f = fb.feedback
        node.get_logger().info(
            f"[{f.phase}] planner_state='{f.planner_state}' frac={f.fraction_complete:.2f}")

    send = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        node.get_logger().error("goal REJECTED (check frame_id == base_link)")
        return 2
    res_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    res = res_future.result().result
    node.get_logger().info(f"result error_code={res.error_code} msg='{res.error_string}'")
    rclpy.shutdown()
    return 0 if res.error_code == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 6: Commit**
```sh
git add kinova_arm_ros2/src/bringup_node.cpp kinova_arm_ros2/CMakeLists.txt kinova_arm_ros2/test/send_goto_pose.py
git commit   # feat(ros2): host GoToEEPose in kinova_arm_node (MultiThreadedExecutor) + client
```

---

### Task 8: Documentation

**Files:**
- Create: `kinova_arm_ros2/docs/guide-goto-ee-pose.md`
- Modify: `kinova_arm_ros2/README.md`

**Interfaces:** none (docs).

- [ ] **Step 1: Write the guide page**

`kinova_arm_ros2/docs/guide-goto-ee-pose.md` — cover, in prose + code:
  - **What it is:** `GoToEEPose` moves the tool to a `base_link` pose; cuRobo plans collision-free, our node executes the plan through the same `Supervisor` used by `ExecuteJointTrajectory`. Two nodes total: ours + the external `rammp_curobo` node.
  - **Bring-up:** launch the cuRobo planner (`ros2 launch rammp_curobo_ros planner.launch.py config:=gen3_real.yaml` — planning-only, no `execute:=true` needed since we execute, not cuRobo) alongside `kinova_arm_node`. cuRobo reads the `/joint_states` our node publishes for the plan start state.
  - **Call it:** `python3 test/send_goto_pose.py --pos X Y Z --quat X Y Z W` (base_link, xyzw). Pick a pose near the current tool pose for a safe local move.
  - **Result codes:** `0` SUCCESSFUL, `-1` INVALID_GOAL (e.g. frame_id ≠ base_link), `-4` PATH_TOLERANCE_VIOLATED, `-6` PREEMPTED, `-7` PLANNING_FAILED (cuRobo returned no plan / unavailable / busy). Feedback has two phases: `planning` (relays cuRobo `state`) then `executing` (`fraction_complete` + live `actual`).
  - **Safety:** the trajectory executes at cuRobo's **full planned speed** (no speed_scale in v1). First real-arm goals: small/near target, attended, e-stop in hand — see `docs/on-robot-runbook.md`.
  - **Design record:** link `docs/superpowers/specs/2026-08-14-goto-ee-pose-curobo-design.md`.

- [ ] **Step 2: Add a `GoToEEPose` line to the README**

In `kinova_arm_ros2/README.md`, add `/go_to_ee_pose` to the list of actions the node hosts, with a one-line pointer to `docs/guide-goto-ee-pose.md`.

- [ ] **Step 3: Commit**
```sh
git add kinova_arm_ros2/docs/guide-goto-ee-pose.md kinova_arm_ros2/README.md
git commit   # docs(ros2): GoToEEPose usage guide + README pointer
```

---

## Validation milestones (operational — run after Task 8, not code tasks)

- **D — sim e2e with the REAL cuRobo node (GPU, no arm motion):** on abra, launch the real `rammp_curobo` planner + our `kinova_arm_node --sim`. Send `send_goto_pose.py` with a base_link target near the sim tool pose. Expect `error_code=0`, feedback progressing `planning`→`executing`. The arm never moves (SimTransport). This validates the real cuRobo contract end-to-end. Always `pkill -TERM -f kinova_arm_node` after and verify it stopped.
- **E — attended real-arm run:** real cuRobo + `kinova_arm_node --ip <arm>` (KORTEX build: add `--cmake-args -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64` to the build). Dry-run/read-only first (confirm `/joint_states` shows the REAL pose), then a small/near target, slow, e-stop in hand, per `docs/on-robot-runbook.md`. Gated behind D. Never unattended.

## Branch integration (after all tasks + D/E)

- Core: open a PR for `feat/planning-failed-result-code` (one-line enum). Merge before/with the ROS2 PR so `.repos` (main) resolves.
- ROS2: open a PR for `feat/goto-ee-pose-curobo`. Per handoff §7, a whole-branch review precedes merge.
