# ROS2 Backend — ExecuteJointTrajectory Implementation Plan (Plan 3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the `kinova_gen3_ros2` ROS2 Humble frontend so a ROS2 action client can send a full `ExecuteJointTrajectory` goal and have the driver execute it — proven end-to-end against `SimTransport`, then built for and run on the real arm (attended).

**Architecture:** Two ament packages in this repo. `kinova_gen3_interfaces` defines the custom action + gains msg (rosidl). `kinova_gen3_ros2` holds `Ros2Backend` (the only unit that includes rclcpp — it owns an rclcpp action server whose callbacks call the driver's `CommandSink`, and it implements the driver's driven ports `ActionServerPort`/`StreamPort` to push feedback/results back out) plus a DI bring-up node that wires the transport → `FeedbackTap` → `RtExecutor` + modes + `Supervisor` + `Ros2Backend` and spins. The core driver (`kinova_lowlevel`) is vendored into a colcon workspace and consumed via `find_package(kinova_lowlevel CONFIG)`. Nothing new runs on the 1 kHz RT thread.

**Tech Stack:** ROS2 Humble, rclcpp / rclcpp_action, rosidl, colcon/ament_cmake, C++17, rclpy (test client), gtest. Builds on aarch64 (abra) only.

## Global Constraints

- **Builds ONLY on abra (aarch64) in a colcon workspace.** muk cannot build. The dev loop rsyncs muk→abra and runs colcon; see Task 1's `scripts/abra_colcon.sh`.
- **The core is vendored, not fetched.** abra has no GitHub key; the deploy script rsyncs the core working tree (muk `~/atdev/kinova-gen3-driver`, branch `feat/interface-supervisor-ports` — Plan 2, carries the b1 export + Supervisor) into the workspace `src/`. The `.repos` file documents the eventual GitHub source; flip its ref to `main` after Plan 2 merges.
- **colcon + pinocchio prefix:** always `export CMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix:$CMAKE_PREFIX_PATH` before `colcon build` (colcon APPENDS its workspace overlays to this; it never passes `-DCMAKE_PREFIX_PATH`, so the ROS underlay stays visible). Never pass the pinocchio prefix via `--cmake-args`.
- **`GoalId` ≡ `GoalUUID`:** the core's `kinova::interface::GoalId` is `std::array<uint8_t,16>`; rclcpp_action's `GoalUUID` is `std::array<uint8_t,16>`. They are the same type — copy directly.
- **Result codes** (core `kinova::interface::result_code`): `kSuccessful=0`, `kInvalidGoal=-1`, `kPathToleranceViolated=-4`, `kGoalToleranceViolated=-5`, `kPreempted=-6`. The action Result `error_code` carries these verbatim.
- **RT-safety unchanged:** the Ros2Backend, its action callbacks, and the rclcpp executor are all non-RT; they reach the RT loop only through the Supervisor's existing lock-free seams. Do not add anything to `compute`/the executor cycle.
- **Scope:** the `ExecuteJointTrajectory` action path only. `JointState` stream (`StreamPort`) is a no-op stub here; `set_gains`/`query_state` services are a fast-follow plan.
- **Real-arm build (Task 7):** `colcon build --cmake-args -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64` (the ament packages ignore the unknown define; the core compiles `KortexTransport`). abra is build+run host, so the KORTEX absolute path baked into the exported target is valid.

______________________________________________________________________

## Core interfaces consumed (verbatim, from Plan 2 — do not redefine)

```cpp
// kinova_lowlevel/interface/value_types.h  (namespace kinova::interface)
using GoalId = std::array<uint8_t, 16>;
struct JointImpedanceGains { JointVec kq; double zeta; JointVec torque_limit; };
struct TrajectoryGoal { Trajectory trajectory; JointVec path_tolerance /*<0 disables*/; JointVec goal_tolerance;
                        double goal_time_tolerance_s; ControlModeKind control_mode; Preemption preemption;
                        JointImpedanceGains gains; bool has_gains; std::string sender_id; };
struct TrajectoryFeedback { JointVec desired, actual, error; double fraction_complete; };
struct TrajectoryResult   { int error_code; std::string error_string; JointVec final_error; };
struct ArmState { JointVec q, qd, tau; Pose ee_pose; bool fault; double stamp_s; };
enum class GoalResponse { kAccept, kReject };
enum class CancelResponse { kAccept, kReject };
namespace result_code { constexpr int kSuccessful=0,kInvalidGoal=-1,kPathToleranceViolated=-4,
                                      kGoalToleranceViolated=-5,kPreempted=-6; }
// kinova_lowlevel/interface/trajectory_executor.h
struct JointWaypoint { JointVec q; double t_s; };
struct Trajectory { std::vector<JointWaypoint> points; double duration_s() const; };
enum class Preemption { kQueue, kLatestWins };
enum class ControlModeKind { kPosition, kImpedance };
// kinova_lowlevel/interface/ports.h
class StreamPort { public: virtual ~StreamPort()=default; virtual void publish_state(const ArmState&)=0; };
class ActionServerPort { public: virtual ~ActionServerPort()=default;
  virtual void publish_feedback(const GoalId&, const TrajectoryFeedback&)=0;
  virtual void settle(const GoalId&, const TrajectoryResult&)=0; };
class CommandSink { public: virtual ~CommandSink()=default;
  virtual GoalResponse   on_trajectory_goal(const TrajectoryGoal&)=0;
  virtual void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&)=0;
  virtual CancelResponse on_trajectory_cancel(const GoalId&)=0;
  virtual GainsResult    on_set_gains(const GainsRequest&)=0;
  virtual ArmState       on_query_state()=0; };
// kinova_lowlevel/interface/supervisor.h
class Supervisor : public CommandSink {
  Supervisor(JointPositionMode&, JointImpedanceMode&, RtExecutor&, Seqlock<JointFeedback>&, Dynamics& pump_dyn,
             StreamPort&, ActionServerPort&, SupervisorConfig={});
  void start(); void stop(); };
// kinova_lowlevel/{joint_types,cartesian_types,dynamics,rt_executor,sim_transport,feedback_tap,
//                  joint_position_mode,joint_impedance_mode,telemetry}.h  (as used by trajectory_run)
```

rclcpp_action Humble (verbatim, source-verified):

```cpp
using GoalUUID = std::array<uint8_t, 16>;
enum class GoalResponse : int8_t { REJECT=1, ACCEPT_AND_EXECUTE=2, ACCEPT_AND_DEFER=3 };
enum class CancelResponse : int8_t { REJECT=1, ACCEPT=2 };
// callbacks: handle_goal(const GoalUUID&, shared_ptr<const Action::Goal>) -> rclcpp_action::GoalResponse
//            handle_cancel(shared_ptr<ServerGoalHandle<Action>>) -> rclcpp_action::CancelResponse
//            handle_accepted(shared_ptr<ServerGoalHandle<Action>>)
// goal_handle: publish_feedback(shared_ptr<Feedback>), succeed(Result::SharedPtr), abort(...), canceled(...),
//              is_canceling(), get_goal(), get_goal_id() -> const GoalUUID&
// create_server<Action>(node, "name", handle_goal, handle_cancel, handle_accepted)
```

______________________________________________________________________

## File structure (all under this repo `kinova_gen3_ros2/`)

- `scripts/abra_colcon.sh` — rsync muk→abra + colcon build (+ optional gtest / node-launch test) (NEW)
- `kinova_gen3.repos` — vcs source list documenting the core's GitHub origin (NEW)
- `README.md` — build/run instructions (NEW)
- `kinova_gen3_interfaces/{package.xml,CMakeLists.txt,action/ExecuteJointTrajectory.action,msg/JointImpedanceGains.msg}` (NEW)
- `kinova_gen3_ros2/package.xml`, `kinova_gen3_ros2/CMakeLists.txt` (NEW)
- `kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h` + `src/message_mapping.cpp` — pure value-type↔msg field copies (NEW)
- `kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h` + `src/ros2_backend.cpp` — action server + driven ports (NEW)
- `kinova_gen3_ros2/src/bringup_node.cpp` — DI wiring + spin (NEW)
- `kinova_gen3_ros2/test/message_mapping_test.cpp` — gtest for the mapping (NEW)
- `kinova_gen3_ros2/test/send_trajectory.py` — rclpy action client (integration harness) (NEW)

______________________________________________________________________

### Task 1: Colcon workspace + deploy loop + vendored-core build (interop spike)

**Files:**

- Create: `scripts/abra_colcon.sh`, `kinova_gen3.repos`, `README.md`

**Interfaces:**

- Produces: `scripts/abra_colcon.sh` — rsyncs the core (muk `~/atdev/kinova-gen3-driver`) → `abra:/tmp/kinova-ros2-ws/src/kinova-gen3-driver` and this repo → `abra:/tmp/kinova-ros2-ws/src/kinova_gen3_ros2`, then runs `colcon build` (with the pinocchio prefix on `CMAKE_PREFIX_PATH`). Accepts `--packages-select <pkg>` passthrough and optional extra colcon args.

- [ ] **Step 1: Write the deploy script**

```bash
# scripts/abra_colcon.sh
#!/usr/bin/env bash
# Deploy loop: rsync muk -> abra colcon workspace, colcon build, optional test.
# Usage: abra_colcon.sh [colcon-args...]   e.g.  abra_colcon.sh --packages-select kinova_lowlevel
set -euo pipefail
CORE_SRC="/home/swapnil/atdev/kinova-gen3-driver/"          # Plan 2 branch working tree
ROS_SRC="/home/swapnil/atdev/kinova_gen3_ros2/"
WS="/tmp/kinova-ros2-ws"
CMEEL="/usr/local/lib/python3.10/dist-packages/cmeel.prefix"

rsync -az --delete --exclude '.git/' --exclude 'build/' --exclude 'build_kortex/' --exclude 'site/' \
  "$CORE_SRC" "abra:$WS/src/kinova-gen3-driver/"
rsync -az --delete --exclude '.git/' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "$ROS_SRC" "abra:$WS/src/kinova_gen3_ros2/"

ssh abra "bash -lc '
  set -euo pipefail
  source /opt/ros/humble/setup.bash
  export CMAKE_PREFIX_PATH=$CMEEL:\$CMAKE_PREFIX_PATH
  cd $WS
  colcon build --event-handlers console_direct+ $* 2>&1 | tail -40
'"
```

- [ ] **Step 2: Write `kinova_gen3.repos` and README**

```yaml
# kinova_gen3.repos — colcon workspace source list.
# NOTE (2026-08): abra has no GitHub key, so the dev loop rsyncs the core in via
# scripts/abra_colcon.sh instead of vcs-importing this. This file documents the
# intended source; pin `version` to main once Plan 2 (feat/interface-supervisor-ports) merges.
repositories:
  kinova-gen3-driver:
    type: git
    url: git@github.com:rammp-org/kinova-gen3-driver.git
    version: feat/interface-supervisor-ports
```

```markdown
# kinova_gen3_ros2
ROS2 Humble frontend for the Kinova Gen3 low-level driver. See
`docs/superpowers/specs/2026-08-12-ros2-backend-realization-design.md`.
Build (aarch64 / abra colcon workspace): `scripts/abra_colcon.sh`.
```

- [ ] **Step 3: Run the vendored-core build (verify the interop spike)**

Run: `bash scripts/abra_colcon.sh --packages-select kinova_lowlevel`
Expected: `colcon build` succeeds building `kinova_lowlevel` as a `build_type cmake` package.

- [ ] **Step 4: Verify the core's exported config landed in the overlay**

Run: `ssh abra 'ls /tmp/kinova-ros2-ws/install/kinova_lowlevel/lib/cmake/kinova_lowlevel/kinova_lowlevelConfig.cmake'`
Expected: the file exists (downstream `find_package(kinova_lowlevel CONFIG)` will resolve it).

- [ ] **Step 5: Commit**

```bash
git add scripts/abra_colcon.sh kinova_gen3.repos README.md
git commit -m "build: colcon deploy loop + vendored-core build (interop spike)"
```

______________________________________________________________________

### Task 2: `kinova_gen3_interfaces` package (the action + gains msg)

**Files:**

- Create: `kinova_gen3_interfaces/package.xml`, `kinova_gen3_interfaces/CMakeLists.txt`,
  `kinova_gen3_interfaces/action/ExecuteJointTrajectory.action`, `kinova_gen3_interfaces/msg/JointImpedanceGains.msg`

**Interfaces:**

- Produces: the action type `kinova_gen3_interfaces::action::ExecuteJointTrajectory` (C++ header `kinova_gen3_interfaces/action/execute_joint_trajectory.hpp`) and `kinova_gen3_interfaces::msg::JointImpedanceGains`, with the field layout below.

- [ ] **Step 1: Write the action + msg**

```
# kinova_gen3_interfaces/action/ExecuteJointTrajectory.action
# ---------- Goal ----------
trajectory_msgs/JointTrajectory       trajectory
control_msgs/JointTolerance[]         path_tolerance
control_msgs/JointTolerance[]         goal_tolerance
builtin_interfaces/Duration           goal_time_tolerance
uint8   control_mode        # 0 = POSITION, 1 = IMPEDANCE
uint8   preemption          # 0 = QUEUE,    1 = LATEST_WINS
JointImpedanceGains gains   # used iff control_mode == IMPEDANCE
string  sender_id
---
# ---------- Result ----------
int32   error_code          # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4, GOAL_TOLERANCE_VIOLATED=-5, PREEMPTED=-6
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error
---
# ---------- Feedback ----------
std_msgs/Header                        header
string[]                               joint_names
trajectory_msgs/JointTrajectoryPoint   desired
trajectory_msgs/JointTrajectoryPoint   actual
trajectory_msgs/JointTrajectoryPoint   error
float32                                fraction_complete
```

```
# kinova_gen3_interfaces/msg/JointImpedanceGains.msg
float64[7] kq
float64    zeta
float64[7] torque_limit
```

- [ ] **Step 2: Write `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>kinova_gen3_interfaces</name>
  <version>0.1.0</version>
  <description>Kinova arm ROS2 action/msg interfaces</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>action_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>std_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>control_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 3: Write `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointTrajectory.action"
  "msg/JointImpedanceGains.msg"
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs control_msgs action_msgs)
ament_package()
```

- [ ] **Step 4: Build and verify generation**

Run: `bash scripts/abra_colcon.sh --packages-select kinova_gen3_interfaces`
Expected: build succeeds; then
`ssh abra 'source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash; ros2 interface show kinova_gen3_interfaces/action/ExecuteJointTrajectory'`
Expected: prints the goal/result/feedback field layout above.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_interfaces
git commit -m "feat(interfaces): ExecuteJointTrajectory.action + JointImpedanceGains.msg"
```

______________________________________________________________________

### Task 3: Message mapping (pure value-type ↔ ROS2 field copies, gtest)

**Files:**

- Create: `kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h`, `kinova_gen3_ros2/src/message_mapping.cpp`,
  `kinova_gen3_ros2/test/message_mapping_test.cpp`
- Create (partial, extended in Task 4): `kinova_gen3_ros2/package.xml`, `kinova_gen3_ros2/CMakeLists.txt`

**Interfaces:**

- Produces (namespace `kinova_gen3_ros2`):
  - `kinova::interface::TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal&)`
  - `ExecuteJointTrajectory::Feedback to_feedback_msg(const GoalId&, const kinova::interface::TrajectoryFeedback&)`
  - `ExecuteJointTrajectory::Result to_result_msg(const kinova::interface::TrajectoryResult&)`
- Consumes: Task 2's message types; core value types.

**Mapping rules (v1):** joints are in the driver's canonical order (7 DOF); `positions` size must be 7. `path_tolerance`/`goal_tolerance`: if the `JointTolerance[]` array is empty → `JointVec::Constant(-1)` (guard disabled); else take each element's `.position` in order into a `JointVec`. `control_mode`: 0→kPosition, 1→kImpedance. `preemption`: 0→kQueue, 1→kLatestWins. `has_gains = (control_mode == 1)`. `time_from_start` → seconds (`sec + nanosec*1e-9`).

- [ ] **Step 1: Write the failing test**

```cpp
// kinova_gen3_ros2/test/message_mapping_test.cpp
#include <gtest/gtest.h>
#include "kinova_gen3_ros2/message_mapping.h"
using namespace kinova_gen3_ros2;
using kinova::interface::ControlModeKind; using kinova::interface::Preemption;

static trajectory_msgs::msg::JointTrajectoryPoint pt(double v, double t) {
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.assign(7, v);
  p.time_from_start.sec = static_cast<int32_t>(t);
  p.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
  return p;
}

TEST(MessageMapping, GoalToTrajectoryGoalPosition) {
  kinova_gen3_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points = { pt(0.0, 0.0), pt(0.5, 2.0) };
  g.control_mode = 0;      // POSITION
  g.preemption   = 1;      // LATEST_WINS
  // path_tolerance empty -> guard disabled (-1)
  auto tg = to_trajectory_goal(g);
  EXPECT_EQ(tg.trajectory.points.size(), 2u);
  EXPECT_NEAR(tg.trajectory.points[1].q[0], 0.5, 1e-12);
  EXPECT_NEAR(tg.trajectory.points[1].t_s, 2.0, 1e-9);
  EXPECT_EQ(tg.control_mode, ControlModeKind::kPosition);
  EXPECT_EQ(tg.preemption, Preemption::kLatestWins);
  EXPECT_LT(tg.path_tolerance[0], 0.0);       // disabled
  EXPECT_FALSE(tg.has_gains);
}

TEST(MessageMapping, GoalImpedanceGainsAndPathTol) {
  kinova_gen3_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points = { pt(0.0, 0.0), pt(0.1, 1.0) };
  g.control_mode = 1;      // IMPEDANCE
  for (int i = 0; i < 7; ++i) g.gains.kq[i] = 60.0;
  g.gains.zeta = 0.6;
  for (int i = 0; i < 7; ++i) g.gains.torque_limit[i] = 9.0;
  control_msgs::msg::JointTolerance jt; jt.position = 0.2;
  g.path_tolerance.assign(7, jt);
  auto tg = to_trajectory_goal(g);
  EXPECT_TRUE(tg.has_gains);
  EXPECT_NEAR(tg.gains.kq[0], 60.0, 1e-12);
  EXPECT_NEAR(tg.gains.zeta, 0.6, 1e-12);
  EXPECT_NEAR(tg.path_tolerance[0], 0.2, 1e-12);
}

TEST(MessageMapping, ResultCarriesErrorCode) {
  kinova::interface::TrajectoryResult r; r.error_code = -4; r.error_string = "path tol";
  r.final_error = kinova::JointVec::Constant(0.01);
  auto m = to_result_msg(r);
  EXPECT_EQ(m.error_code, -4);
  EXPECT_EQ(m.error_string, "path tol");
  ASSERT_EQ(m.final_error.positions.size(), 7u);
  EXPECT_NEAR(m.final_error.positions[0], 0.01, 1e-12);
}
```

- [ ] **Step 2: Write the package skeleton so the test can build**

`kinova_gen3_ros2/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>kinova_gen3_ros2</name>
  <version>0.1.0</version>
  <description>ROS2 frontend node for the Kinova low-level driver</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>kinova_gen3_interfaces</depend>
  <depend>kinova_lowlevel</depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

`kinova_gen3_ros2/CMakeLists.txt` (mapping lib + test only for now; node target added in Task 4):

```cmake
cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_ros2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kinova_gen3_interfaces REQUIRED)
find_package(kinova_lowlevel CONFIG REQUIRED)

add_library(message_mapping src/message_mapping.cpp)
target_include_directories(message_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(message_mapping rclcpp kinova_gen3_interfaces)
target_link_libraries(message_mapping kinova_lowlevel::kinova_lowlevel)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(message_mapping_test test/message_mapping_test.cpp)
  target_link_libraries(message_mapping_test message_mapping)
  ament_target_dependencies(message_mapping_test kinova_gen3_interfaces)
endif()

ament_package()
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `bash scripts/abra_colcon.sh --packages-select kinova_gen3_ros2` then
`ssh abra 'cd /tmp/kinova-ros2-ws && colcon test --packages-select kinova_gen3_ros2 --event-handlers console_direct+ && colcon test-result --verbose'`
Expected: FAIL — `message_mapping.h` not found (compile error).

- [ ] **Step 4: Write the mapping**

```cpp
// kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h
#pragma once
#include "kinova_gen3_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova_gen3_ros2 {
using ExecuteJointTrajectory = kinova_gen3_interfaces::action::ExecuteJointTrajectory;
kinova::interface::TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g);
ExecuteJointTrajectory::Feedback to_feedback_msg(const kinova::interface::GoalId& id,
                                                 const kinova::interface::TrajectoryFeedback& fb);
ExecuteJointTrajectory::Result to_result_msg(const kinova::interface::TrajectoryResult& r);
}  // namespace kinova_gen3_ros2
```

```cpp
// kinova_gen3_ros2/src/message_mapping.cpp
#include "kinova_gen3_ros2/message_mapping.h"
namespace kinova_gen3_ros2 {
using namespace kinova; using namespace kinova::interface;

static JointVec tol_to_vec(const std::vector<control_msgs::msg::JointTolerance>& t) {
  if (t.empty()) return JointVec::Constant(-1.0);           // disabled
  JointVec v = JointVec::Constant(-1.0);
  for (int i = 0; i < kNumJoints && i < static_cast<int>(t.size()); ++i) v[i] = t[i].position;
  return v;
}

TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g) {
  TrajectoryGoal tg;
  for (const auto& p : g.trajectory.points) {
    JointWaypoint w;
    for (int i = 0; i < kNumJoints && i < static_cast<int>(p.positions.size()); ++i) w.q[i] = p.positions[i];
    w.t_s = static_cast<double>(p.time_from_start.sec) + static_cast<double>(p.time_from_start.nanosec) * 1e-9;
    tg.trajectory.points.push_back(w);
  }
  tg.path_tolerance = tol_to_vec(g.path_tolerance);
  tg.goal_tolerance = tol_to_vec(g.goal_tolerance);
  tg.goal_time_tolerance_s = static_cast<double>(g.goal_time_tolerance.sec)
                           + static_cast<double>(g.goal_time_tolerance.nanosec) * 1e-9;
  tg.control_mode = (g.control_mode == 1) ? ControlModeKind::kImpedance : ControlModeKind::kPosition;
  tg.preemption   = (g.preemption == 1)   ? Preemption::kLatestWins    : Preemption::kQueue;
  tg.has_gains = (g.control_mode == 1);
  if (tg.has_gains) {
    for (int i = 0; i < kNumJoints; ++i) { tg.gains.kq[i] = g.gains.kq[i]; tg.gains.torque_limit[i] = g.gains.torque_limit[i]; }
    tg.gains.zeta = g.gains.zeta;
  }
  tg.sender_id = g.sender_id;
  return tg;
}

static trajectory_msgs::msg::JointTrajectoryPoint vec_to_point(const JointVec& v) {
  trajectory_msgs::msg::JointTrajectoryPoint p; p.positions.resize(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) p.positions[i] = v[i];
  return p;
}

ExecuteJointTrajectory::Feedback to_feedback_msg(const GoalId&, const TrajectoryFeedback& fb) {
  ExecuteJointTrajectory::Feedback m;
  m.desired = vec_to_point(fb.desired);
  m.actual  = vec_to_point(fb.actual);
  m.error   = vec_to_point(fb.error);
  m.fraction_complete = static_cast<float>(fb.fraction_complete);
  return m;
}

ExecuteJointTrajectory::Result to_result_msg(const TrajectoryResult& r) {
  ExecuteJointTrajectory::Result m;
  m.error_code = r.error_code; m.error_string = r.error_string;
  m.final_error = vec_to_point(r.final_error);
  return m;
}
}  // namespace kinova_gen3_ros2
```

- [ ] **Step 5: Run the test to verify it passes**

Run: same as Step 3.
Expected: PASS (3 tests).

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_ros2/package.xml kinova_gen3_ros2/CMakeLists.txt \
        kinova_gen3_ros2/include/kinova_gen3_ros2/message_mapping.h \
        kinova_gen3_ros2/src/message_mapping.cpp kinova_gen3_ros2/test/message_mapping_test.cpp
git commit -m "feat(ros2): value-type <-> ExecuteJointTrajectory message mapping (gtest)"
```

______________________________________________________________________

### Task 4: `Ros2Backend` — action server + driven ports

**Files:**

- Create: `kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h`, `kinova_gen3_ros2/src/ros2_backend.cpp`
- Modify: `kinova_gen3_ros2/CMakeLists.txt` (add the backend to a library/target)

**Interfaces:**

- Produces: `class Ros2Backend : public kinova::interface::ActionServerPort, public kinova::interface::StreamPort`.
  Constructor `Ros2Backend(rclcpp::Node::SharedPtr node)` — creates the action server `"execute_joint_trajectory"`.
  `void set_command_sink(kinova::interface::CommandSink* sink)`. Implements `publish_feedback`, `settle` (driven, called by the supervisor's sampler thread) and `publish_state` (no-op stub for v1). Callbacks translate ROS goals → `CommandSink`.

**Threading:** the action callbacks run on the rclcpp executor thread; the driven ports (`publish_feedback`/`settle`) are called by the supervisor's sampler thread. The `GoalId → goal handle` map is guarded by a mutex. Calling `goal_handle->succeed/publish_feedback` from the sampler thread is the sanctioned rclcpp_action pattern (each call is internally atomic; the supervisor is the sole per-goal caller).

- [ ] **Step 1: Write the header**

```cpp
// kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_gen3_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_gen3_ros2/message_mapping.h"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

class Ros2Backend : public kinova::interface::ActionServerPort,
                    public kinova::interface::StreamPort {
 public:
  using Action = kinova_gen3_interfaces::action::ExecuteJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  explicit Ros2Backend(rclcpp::Node::SharedPtr node);
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

  // ActionServerPort (called by the supervisor sampler thread):
  void publish_feedback(const kinova::interface::GoalId&, const kinova::interface::TrajectoryFeedback&) override;
  void settle(const kinova::interface::GoalId&, const kinova::interface::TrajectoryResult&) override;
  // StreamPort (JointState stream is a fast-follow plan — no-op in v1):
  void publish_state(const kinova::interface::ArmState&) override {}

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
  void handle_accepted(std::shared_ptr<GoalHandle>);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  kinova::interface::CommandSink* sink_ = nullptr;
  std::mutex m_;
  std::map<kinova::interface::GoalId, std::shared_ptr<GoalHandle>> handles_;   // GoalId == GoalUUID
};
}  // namespace kinova_gen3_ros2
```

- [ ] **Step 2: Write the implementation**

```cpp
// kinova_gen3_ros2/src/ros2_backend.cpp
#include "kinova_gen3_ros2/ros2_backend.h"
#include <functional>
namespace kinova_gen3_ros2 {
using namespace kinova::interface;
using std::placeholders::_1; using std::placeholders::_2;

Ros2Backend::Ros2Backend(rclcpp::Node::SharedPtr node) : node_(node) {
  server_ = rclcpp_action::create_server<Action>(
      node_, "execute_joint_trajectory",
      std::bind(&Ros2Backend::handle_goal, this, _1, _2),
      std::bind(&Ros2Backend::handle_cancel, this, _1),
      std::bind(&Ros2Backend::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse Ros2Backend::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
  if (!sink_) return rclcpp_action::GoalResponse::REJECT;
  const GoalResponse r = sink_->on_trajectory_goal(to_trajectory_goal(*goal));
  return (r == GoalResponse::kAccept) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                                      : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse Ros2Backend::handle_cancel(std::shared_ptr<GoalHandle> gh) {
  if (!sink_) return rclcpp_action::CancelResponse::REJECT;
  const CancelResponse r = sink_->on_trajectory_cancel(gh->get_goal_id());   // GoalUUID -> GoalId
  return (r == CancelResponse::kAccept) ? rclcpp_action::CancelResponse::ACCEPT
                                        : rclcpp_action::CancelResponse::REJECT;
}

void Ros2Backend::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  { std::lock_guard<std::mutex> l(m_); handles_[id] = gh; }
  sink_->on_trajectory_accepted(id, to_trajectory_goal(*gh->get_goal()));
}

void Ros2Backend::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second; }
  auto msg = std::make_shared<Action::Feedback>(to_feedback_msg(id, fb));
  gh->publish_feedback(msg);
}

void Ros2Backend::settle(const GoalId& id, const TrajectoryResult& r) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second; handles_.erase(it); }
  auto msg = std::make_shared<Action::Result>(to_result_msg(r));
  if (gh->is_canceling())                      gh->canceled(msg);
  else if (r.error_code == result_code::kSuccessful) gh->succeed(msg);
  else                                         gh->abort(msg);
}
}  // namespace kinova_gen3_ros2
```

- [ ] **Step 3: Add the backend to CMake and build**

In `kinova_gen3_ros2/CMakeLists.txt`, add after the `message_mapping` library:

```cmake
add_library(ros2_backend src/ros2_backend.cpp)
target_include_directories(ros2_backend PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ros2_backend rclcpp rclcpp_action kinova_gen3_interfaces)
target_link_libraries(ros2_backend message_mapping kinova_lowlevel::kinova_lowlevel)
```

Run: `bash scripts/abra_colcon.sh --packages-select kinova_gen3_ros2`
Expected: builds clean (backend compiles + links against rclcpp_action, the interfaces, and the core).

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h kinova_gen3_ros2/src/ros2_backend.cpp kinova_gen3_ros2/CMakeLists.txt
git commit -m "feat(ros2): Ros2Backend — action server + driven ports (feedback/settle/cancel)"
```

______________________________________________________________________

### Task 5: Bring-up node — DI wiring + spin (sim)

**Files:**

- Create: `kinova_gen3_ros2/src/bringup_node.cpp`
- Modify: `kinova_gen3_ros2/CMakeLists.txt` (add the `kinova_gen3_node` executable + install)

**Interfaces:**

- Produces: executable `kinova_gen3_node` (`ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf <path>`), wiring `SimTransport`→`FeedbackTap`→`RtExecutor` + `JointPositionMode`+`JointImpedanceMode` + `Supervisor` + `Ros2Backend`, running the RT loop on the main thread and the rclcpp executor + telemetry drain on their own threads.

**Wiring (mirrors `trajectory_run`/`teleop_socket_server`):** construct the node, backend, transport (+FeedbackTap+Seqlock), two `Dynamics` (one for modes, one for the pump), both modes, `SampleRing`, `RtExecutor`; construct the `Supervisor` against the backend's ports; `backend.set_command_sink(&supervisor)`; `supervisor.start()`; spin the rclcpp executor on a thread; run the RT loop on the main thread; SIGINT → stop everything and join.

- [ ] **Step 1: Write the node**

```cpp
// kinova_gen3_ros2/src/bringup_node.cpp
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_ros2/ros2_backend.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif
using namespace kinova;

namespace { std::atomic<bool> g_stop{false}; void on_sigint(int){ g_stop.store(true); } }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string urdf = "models/gen3_7dof_2f85.urdf", ip;
  bool use_sim = false; int cpu = -1, prio = 80; double rate = 1000.0;
  for (int i = 1; i < argc; ++i) { std::string a = argv[i];
    auto nxt = [&]{ return std::string(argv[++i]); };
    if (a == "--sim") use_sim = true; else if (a == "--ip") ip = nxt();
    else if (a == "--urdf") urdf = nxt(); else if (a == "--cpu") cpu = std::stoi(nxt());
    else if (a == "--rt-priority") prio = std::stoi(nxt()); else if (a == "--rate") rate = std::stod(nxt()); }

  Dynamics dyn(urdf), pump_dyn(urdf);
  std::unique_ptr<Transport> base;
  if (use_sim) { JointFeedback init; base = std::make_unique<SimTransport>(init); }
  else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) { RCLCPP_ERROR(rclcpp::get_logger("kinova_gen3_node"), "real mode needs --ip"); return 2; }
    base = std::make_unique<KortexTransport>(ip);
#else
    RCLCPP_ERROR(rclcpp::get_logger("kinova_gen3_node"), "built without KORTEX; use --sim"); return 2;
#endif
  }
  Seqlock<JointFeedback> snap; FeedbackTap tap(*base, snap);

  JointPositionMode pos(dyn); JointImpedanceMode imp(dyn);
  SampleRing ring(1u << 16);
  RtExecutor exec(tap, ring, {rate, Pacing::kSleepSpin, {prio, cpu, true}});

  auto node = std::make_shared<rclcpp::Node>("kinova_gen3_node");
  auto backend = std::make_shared<kinova_gen3_ros2::Ros2Backend>(node);
  interface::Supervisor sup(pos, imp, exec, snap, pump_dyn, *backend, *backend);
  backend->set_command_sink(&sup);

  std::signal(SIGINT, on_sigint);
  tap.connect(); tap.set_servoing_low_level();
  sup.start();

  std::thread ros_spin([&]{ rclcpp::executors::SingleThreadedExecutor ex; ex.add_node(node);
    while (!g_stop.load() && rclcpp::ok()) ex.spin_some(std::chrono::milliseconds(10)); });
  std::thread drain([&]{ CycleSample s; while (!g_stop.load()) { while (ring.pop(s)) {} std::this_thread::sleep_for(std::chrono::milliseconds(5)); } while (ring.pop(s)) {} });

  RCLCPP_INFO(node->get_logger(), "kinova_gen3_node up (%s); action: /execute_joint_trajectory", use_sim ? "sim" : "real");
  exec.run(g_stop);            // RT loop on the main thread; returns when g_stop set

  sup.stop(); base->safe_shutdown();
  ros_spin.join(); drain.join(); rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Add the executable to CMake and build**

In `kinova_gen3_ros2/CMakeLists.txt`, add:

```cmake
add_executable(kinova_gen3_node src/bringup_node.cpp)
ament_target_dependencies(kinova_gen3_node rclcpp rclcpp_action kinova_gen3_interfaces)
target_link_libraries(kinova_gen3_node ros2_backend message_mapping kinova_lowlevel::kinova_lowlevel)
target_compile_definitions(kinova_gen3_node PRIVATE KINOVA_NO_KORTEX)   # sim build; Task 7 flips this
install(TARGETS kinova_gen3_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ../models DESTINATION share/${PROJECT_NAME} OPTIONAL)
```

Run: `bash scripts/abra_colcon.sh --packages-select kinova_gen3_ros2`
Expected: builds clean.

- [ ] **Step 3: Smoke-test the node starts and advertises the action**

Run:

```
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash;
  cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver;
  timeout 6 ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf & sleep 3;
  ros2 action list; kill %1 2>/dev/null"'
```

Expected: `/execute_joint_trajectory` appears in `ros2 action list`; node logs "up (sim)"; clean exit.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_ros2/src/bringup_node.cpp kinova_gen3_ros2/CMakeLists.txt
git commit -m "feat(ros2): bring-up node — DI wiring + spin (sim)"
```

______________________________________________________________________

### Task 6: Milestone A — sim end-to-end via Python client

**Files:**

- Create: `kinova_gen3_ros2/test/send_trajectory.py`
- Create: `scripts/abra_e2e_sim.sh` (launch node + run client on abra)

**Interfaces:**

- Produces: a rclpy client that sends an `ExecuteJointTrajectory` goal, prints feedback, and exits nonzero unless the terminal result matches an expected `error_code`. CLI: `send_trajectory.py --mode position --delta 0.05 --dur 0.4 --expect 0` and `--path-tol 0.2 --delta 0.5 --expect -4` (forced divergence).

- [ ] **Step 1: Write the Python client**

```python
#!/usr/bin/env python3
# kinova_gen3_ros2/test/send_trajectory.py
import argparse, sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from kinova_gen3_interfaces.action import ExecuteJointTrajectory

class C(Node):
    def __init__(self, args):
        super().__init__('send_trajectory')
        self.args = args; self.code = None; self.status = None
        self.cli = ActionClient(self, ExecuteJointTrajectory, 'execute_joint_trajectory')

    def run(self):
        self.cli.wait_for_server()
        g = ExecuteJointTrajectory.Goal()
        g.control_mode = 1 if self.args.mode == 'impedance' else 0
        g.preemption = 1  # LATEST_WINS
        p0 = JointTrajectoryPoint(); p0.positions = [0.0]*7; p0.time_from_start = Duration(sec=0)
        p1 = JointTrajectoryPoint(); p1.positions = [0.0]*7; p1.positions[5] = self.args.delta
        s = int(self.args.dur); p1.time_from_start = Duration(sec=s, nanosec=int((self.args.dur-s)*1e9))
        g.trajectory.points = [p0, p1]
        if self.args.path_tol > 0:
            jt = JointTolerance(); jt.position = self.args.path_tol
            g.path_tolerance = [jt]*7
        fut = self.cli.send_goal_async(g, feedback_callback=self.on_fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            print('REJECTED'); self.code = None; return
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        self.code = rf.result().result.error_code; self.status = rf.result().status
        print(f'result error_code={self.code} status={self.status}')

    def on_fb(self, fb):
        print(f'fb fraction={fb.feedback.fraction_complete:.2f}')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='position'); ap.add_argument('--delta', type=float, default=0.05)
    ap.add_argument('--dur', type=float, default=0.4); ap.add_argument('--path-tol', type=float, default=-1.0)
    ap.add_argument('--expect', type=int, required=True)
    a = ap.parse_args()
    rclpy.init(); c = C(a); c.run(); rclpy.shutdown()
    ok = (c.code == a.expect)
    print('PASS' if ok else f'FAIL (got {c.code}, expected {a.expect})')
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Write the launch+client harness**

```bash
# scripts/abra_e2e_sim.sh — run ON abra: launch node (sim), run the client, assert.
#!/usr/bin/env bash
set -uo pipefail
source /opt/ros/humble/setup.bash
source /tmp/kinova-ros2-ws/install/setup.bash
cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver
ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf & NODE=$!
sleep 3
python3 /tmp/kinova-ros2-ws/src/kinova_gen3_ros2/kinova_gen3_ros2/test/send_trajectory.py --mode position --delta 0.05 --dur 0.4 --expect 0
R1=$?
python3 /tmp/kinova-ros2-ws/src/kinova_gen3_ros2/kinova_gen3_ros2/test/send_trajectory.py --delta 0.5 --dur 2.0 --path-tol 0.2 --expect -4
R2=$?
kill $NODE 2>/dev/null; wait $NODE 2>/dev/null
echo "success_case=$R1 divergence_case=$R2"
[ $R1 -eq 0 ] && [ $R2 -eq 0 ]
```

- [ ] **Step 3: Run the end-to-end test**

Run: `bash scripts/abra_colcon.sh` (full build) then `ssh abra 'bash /tmp/kinova-ros2-ws/src/kinova_gen3_ros2/scripts/abra_e2e_sim.sh'`
Expected: the position goal streams feedback (fraction 0→1) and settles `error_code=0` / `STATUS_SUCCEEDED`; the 0.5 rad move against the static sim trips the guard and settles `error_code=-4`. Final line: `success_case=0 divergence_case=0`, script exit 0.

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_ros2/test/send_trajectory.py scripts/abra_e2e_sim.sh
git commit -m "test(ros2): milestone A — sim end-to-end ExecuteJointTrajectory (success + divergence)"
```

______________________________________________________________________

### Task 7: Milestone B — combined KORTEX+ROS2 build

**Files:**

- Modify: `kinova_gen3_ros2/CMakeLists.txt` (make the `KINOVA_NO_KORTEX` compile-def conditional on a build option)

**Interfaces:**

- Produces: the node builds with the real `KortexTransport` compiled in when the workspace is built with `-DKINOVA_ENABLE_KORTEX=ON`, while the default (sim) build is unchanged.

- [ ] **Step 1: Make the node's KORTEX gating match the core's**

In `kinova_gen3_ros2/CMakeLists.txt`, replace the unconditional `target_compile_definitions(kinova_gen3_node PRIVATE KINOVA_NO_KORTEX)` with:

```cmake
option(KINOVA_ENABLE_KORTEX "Link the real KortexTransport path" OFF)
if(NOT KINOVA_ENABLE_KORTEX)
  target_compile_definitions(kinova_gen3_node PRIVATE KINOVA_NO_KORTEX)
endif()
```

(When `-DKINOVA_ENABLE_KORTEX=ON` is passed through colcon, the node compiles the real `--ip` path and links `KortexTransport` from the core, which was itself built with KORTEX enabled.)

- [ ] **Step 2: Run the combined KORTEX+ROS2 build**

Run:

```
bash scripts/abra_colcon.sh --cmake-args -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/home/abra/kortex_api_2.8.0_aarch64
```

Expected: the core compiles `kortex_transport.cpp`; `kinova_gen3_node` links the KORTEX static lib (abra-local abs path, valid) and builds clean.

- [ ] **Step 3: Verify the KORTEX-enabled node still runs in sim**

Run:

```
ssh abra 'bash -lc "source /opt/ros/humble/setup.bash; source /tmp/kinova-ros2-ws/install/setup.bash;
  cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver;
  timeout 6 ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf & sleep 3;
  ros2 action list; kill %1 2>/dev/null"'
```

Expected: the KORTEX-enabled binary still starts in `--sim` and advertises the action (KORTEX compiled but not exercised).

- [ ] **Step 4: Commit**

```bash
git add kinova_gen3_ros2/CMakeLists.txt
git commit -m "build(ros2): combined KORTEX+ROS2 build (node gates KORTEX like the core)"
```

______________________________________________________________________

### Task 8: Milestone C — attended real-arm run (documented; controller + user)

> **This task is NOT subagent-executed.** It is an attended on-robot procedure run by the controller together with the user (e-stop in hand), mirroring the Plan 1 on-robot step. No unattended hardware connection.

**Files:**

- Create: `docs/on-robot-runbook.md` (the procedure below, version-controlled)

**Procedure:**

1. Build the workspace with KORTEX enabled (Task 7 command). Confirm the arm IP is reachable (`ping`), arm powered, homed, workspace clear, e-stop in hand.
1. **Dry-run first (read-only):** launch the node against the arm but send nothing; confirm it connects and advertises the action:
   `ros2 run kinova_gen3_ros2 kinova_gen3_node --ip <arm-ip> --urdf models/gen3_7dof_2f85.urdf`
   then in another shell `ros2 action list` shows `/execute_joint_trajectory`; Ctrl-C.
1. **Small/slow single-joint trajectory (attended):** with the node running against the arm, send a conservative goal from the client — joint 6, ~0.08–0.15 rad, ≥1 s duration, position mode, live path tolerance ~0.2:
   `python3 .../send_trajectory.py --mode position --delta 0.10 --dur 1.2 --path-tol 0.2 --expect 0`
   Watch the arm; confirm feedback streams, the arm reaches the goal, and the result settles `error_code=0`. Re-read measured q (a dry-run) to confirm real motion (per the Plan 1 lesson: the report's reference is commanded, not measured).
1. Record the run (residuals, faults/dropped/majflt from the node's telemetry drain) in `docs/on-robot-runbook.md`.

- [ ] **Step 1: Write `docs/on-robot-runbook.md`** with the procedure above.
- [ ] **Step 2: Commit** `git commit -m "docs(ros2): on-robot runbook (attended real-arm trajectory)"`.
- [ ] **Step 3:** Execute the attended run WITH the user; append the measured result to the runbook and commit.

______________________________________________________________________

## Self-review notes (author)

- **Spec coverage:** realization spec §repo-structure → Tasks 1,2,3–5; §Ros2Backend → Task 4; §bring-up node → Task 5; §combined KORTEX+ROS2 build → Task 7; Milestones A/B/C → Tasks 6/7/8. `.repos` + deploy loop → Task 1. `StreamPort` no-op + services deferred → stated in Task 4 / Global Constraints (out of scope).
- **TDD vs build-gate honesty:** Task 3 (mapping) and Task 6 (end-to-end) are behavioral TDD; Tasks 1,2,4,5,7 are build/smoke gates (infrastructure/wiring — the deliverable is "builds + advertises + runs", verified by `colcon build` + `ros2 action list` + the node starting). Task 8 is attended, not automated.
- **Threading contract:** the backend's driven ports are called only by the supervisor's sampler thread; action callbacks run on the rclcpp executor; the `GoalId→handle` map is mutex-guarded; `succeed/publish_feedback/canceled` from the sampler thread is the sanctioned rclcpp_action pattern. `settle` maps `is_canceling()→canceled`, `kSuccessful→succeed`, else `abort`.
- **Deferred / v1 assumptions (surfaced):** joints assumed in canonical 7-DOF order (no `joint_names` remap); id-agnostic cancel (core design note) mapped onto ROS per-goal cancel — acceptable for single-commander v1; `goal_tolerance`/`goal_time_tolerance` carried but only informational (core completes on time). All flagged for the arbitration / stream follow-on.

```
```
