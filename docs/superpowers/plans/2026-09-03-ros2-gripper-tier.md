# ROS2 Gripper Tier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Command the Robotiq 2F-85 over ROS2 and publish its state, by exposing core's `GripperSink` as `/setpoint/gripper`, `/gripper_state`, and one extra joint in `/joint_states`.

**Architecture:** A `GripperServer` class mirrors `StreamServer` exactly — it holds a `kinova::interface::GripperSink&` and nothing else from core, so it unit-tests against a fake with no robot. `Ros2Backend` gains a `GripperSink*` setter and appends the knuckle joint to the `JointState` it already builds, inheriting that message's stamp. `bringup_node` inserts a `GripperController` into the `Transport` decorator chain and stops passing `deps.grip = nullptr`.

**Tech Stack:** ROS2 Humble, `ament_cmake`, `rosidl`, gtest, rclpy (conformance).

**Spec:** `docs/superpowers/specs/2026-09-03-ros2-gripper-tier-design.md`

## Global Constraints

- **No rclcpp outside `ros2_backend.cpp`, `stream_server.cpp`, `arbitration_server.cpp` and the new `gripper_server.cpp`.** `message_mapping` uses generated message types but never `rclcpp`.
- **No ROS2 header ever reaches the RT thread.**
- **`position`, `speed` and `force` are each in `[0,1]`.** Core's `set_target` clamps; this layer does NOT re-clamp.
- **The knuckle conversion factor is `0.8`** — `robotiq_85_left_knuckle_joint`'s URDF upper limit.
- **The actuated joint name is `robotiq_85_left_knuckle_joint`.** Only this one is published; RSP derives the five mimics.
- **`active` is never exposed on any ROS message.**
- **Gripper `velocity` and `effort` in `/joint_states` are `NaN`**, never 0.
- Commit messages end with the repo's `Co-Authored-By:` / `Claude-Session:` trailers.

---

### Task 1: Interface messages

**Files:**
- Create: `kinova_arm_interfaces/msg/GripperSetpoint.msg`
- Create: `kinova_arm_interfaces/msg/GripperState.msg`
- Modify: `kinova_arm_interfaces/CMakeLists.txt:31` (append to the `rosidl_generate_interfaces` list)

**Interfaces:**
- Consumes: nothing.
- Produces: `kinova_arm_interfaces/msg/GripperSetpoint` with fields `position` (float32), `speed` (float32), `force` (float32), `token` (uint8[16]). `kinova_arm_interfaces/msg/GripperState` with `header` (std_msgs/Header), `position` (float32), `effort` (float32), `current` (float32), `present` (bool).

- [ ] **Step 1: Write `GripperSetpoint.msg`**

```
# Absolute gripper command. Rides the ARM's token -- one physical machine, one holder.
#
# NOT sticky: core's GripperController::set_target takes all three fields every call and
# does not remember the previous speed/force. A message carrying only `position` commands
# speed and force to the DEFAULTS BELOW, it does not leave them alone.
#
# There is deliberately no `active` field. GripperCommand has one, but it belongs to the
# outgoing frame (kortex_transport reads `if (cmd.gripper.active)`), not to the caller:
# set_target arms stamping unconditionally and discards it. A client sending active=false
# to stop commanding the gripper would get it commanded anyway.
#
# There is no "let go". release() does not open the gripper -- the last command stays
# latched in KORTEX's cyclic frame and the 2F-85 is effectively self-locking. To open,
# command position 0.
float32   position    # 0 (open) .. 1 (closed)
float32   speed 1.0   # fraction of max closing speed
# A CEILING on motor current, NOT a force setpoint: the gripper closes at `speed` toward
# `position` and stalls when it reaches this limit. No force servo exists on this hardware.
float32   force 0.5
uint8[16] token       # from /acquire_control; validates the sender, every message
```

- [ ] **Step 2: Write `GripperState.msg`**

```
# What the gripper reports. The units-bearing home for what cannot honestly go in
# sensor_msgs/JointState: see /joint_states, where gripper effort and velocity are NaN.
std_msgs/Header header
float32 position   # 0 (open) .. 1 (closed), normalized
# 0..1, |current| / 1.0 A. NOT Newtons -- MotorFeedback carries no force field.
# A SUSTAINED grasp reports a SMALL effort (~0.05): the current spikes while the fingers
# close on the object, then settles to a low holding current. Anything keying off "high
# effort means holding something" is wrong.
float32 effort
float32 current    # amps, raw, exactly as reported
# False when no gripper is attached OR no GripperController is wired. Core conflates the
# two deliberately: both mean there is nothing to report. Without this, a missing gripper
# and a fully-open one are both position 0.
bool    present
```

- [ ] **Step 3: Register both in CMakeLists.txt**

Add after line 31 (`"srv/ListControllers.srv"`), inside the same `rosidl_generate_interfaces(...)` list:

```cmake
  "msg/GripperSetpoint.msg"
  "msg/GripperState.msg"
```

- [ ] **Step 4: Build and verify the messages generate**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_arm_ros2 && make build 2>&1 | tail -5'
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker run --rm kinova-arm-ros2:humble bash -lc "source /ros2_ws/install/setup.bash && ros2 interface show kinova_arm_interfaces/msg/GripperSetpoint"'
```
Expected: the field list prints, including `uint8[16] token` and NO `active` field.

- [ ] **Step 5: Commit**

```bash
git add kinova_arm_interfaces/msg/GripperSetpoint.msg kinova_arm_interfaces/msg/GripperState.msg kinova_arm_interfaces/CMakeLists.txt
git commit -m "feat(interfaces): gripper setpoint and state messages"
```

---

### Task 2: The normalized <-> radian mapping

**Files:**
- Modify: `kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h`
- Modify: `kinova_arm_ros2/src/message_mapping.cpp`
- Test: `kinova_arm_ros2/test/message_mapping_test.cpp`

**Interfaces:**
- Consumes: `kinova_arm_interfaces/msg/GripperSetpoint`, `kinova_arm_interfaces/msg/GripperState` (Task 1); `kinova::interface::GripperSetpoint`, `kinova::interface::GripperState` from `kinova_lowlevel/interface/value_types.h`.
- Produces:
  - `constexpr double kKnuckleUpperRad = 0.8;`
  - `double gripper_to_knuckle_rad(float normalized);`
  - `kinova::interface::GripperSetpoint to_gripper_setpoint(const kinova_arm_interfaces::msg::GripperSetpoint& m);`
  - `kinova_arm_interfaces::msg::GripperState to_gripper_state_msg(const kinova::interface::GripperState& g);`

- [ ] **Step 1: Write the failing tests**

Append to `kinova_arm_ros2/test/message_mapping_test.cpp`:

```cpp
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"

TEST(GripperMapping, NormalizedMapsOntoTheKnuckleLimits) {
  EXPECT_DOUBLE_EQ(kinova_arm_ros2::gripper_to_knuckle_rad(0.0f), 0.0);
  EXPECT_DOUBLE_EQ(kinova_arm_ros2::gripper_to_knuckle_rad(1.0f), 0.8);
  EXPECT_DOUBLE_EQ(kinova_arm_ros2::gripper_to_knuckle_rad(0.5f), 0.4);
}

// The ROS message has no `active`; core's struct does. to_gripper_setpoint must leave it
// alone rather than inventing a value -- set_target discards it either way, but a caller
// reading the struct should not see a fabricated flag.
TEST(GripperMapping, SetpointCarriesAllThreeFieldsAndTheToken) {
  kinova_arm_interfaces::msg::GripperSetpoint m;
  m.position = 0.25f; m.speed = 0.5f; m.force = 0.75f;
  m.token[0] = 7; m.token[15] = 9;
  const auto s = kinova_arm_ros2::to_gripper_setpoint(m);
  EXPECT_FLOAT_EQ(s.command.position, 0.25f);
  EXPECT_FLOAT_EQ(s.command.speed, 0.5f);
  EXPECT_FLOAT_EQ(s.command.force, 0.75f);
  EXPECT_EQ(s.token[0], 7);
  EXPECT_EQ(s.token[15], 9);
}

TEST(GripperMapping, StateRoundTripsEveryField) {
  kinova::interface::GripperState g;
  g.position = 0.3f; g.effort = 0.05f; g.current = 0.05f; g.present = true;
  const auto m = kinova_arm_ros2::to_gripper_state_msg(g);
  EXPECT_FLOAT_EQ(m.position, 0.3f);
  EXPECT_FLOAT_EQ(m.effort, 0.05f);
  EXPECT_FLOAT_EQ(m.current, 0.05f);
  EXPECT_TRUE(m.present);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker run --rm kinova-arm-ros2:humble bash -lc "cd /ros2_ws && colcon build --packages-select kinova_arm_ros2 2>&1 | tail -20"'
```
Expected: FAIL to compile — `gripper_to_knuckle_rad` is not a member of `kinova_arm_ros2`.

- [ ] **Step 3: Declare the functions in the header**

Add to `kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h`, inside `namespace kinova_arm_ros2`, and add the two `#include`s at the top alongside the existing ones:

```cpp
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"

// robotiq_85_left_knuckle_joint's URDF upper limit. The gripper's ONE actuated DOF;
// robot_state_publisher derives the five mimics from it (verified 2026-09-03).
inline constexpr double kKnuckleUpperRad = 0.8;

// Core reports 0 (open) .. 1 (closed); sensor_msgs/JointState wants radians.
double gripper_to_knuckle_rad(float normalized);

kinova::interface::GripperSetpoint to_gripper_setpoint(
    const kinova_arm_interfaces::msg::GripperSetpoint& m);
kinova_arm_interfaces::msg::GripperState to_gripper_state_msg(
    const kinova::interface::GripperState& g);
```

- [ ] **Step 4: Implement in message_mapping.cpp**

```cpp
double gripper_to_knuckle_rad(float normalized) {
  return static_cast<double>(normalized) * kKnuckleUpperRad;
}

kinova::interface::GripperSetpoint to_gripper_setpoint(
    const kinova_arm_interfaces::msg::GripperSetpoint& m) {
  kinova::interface::GripperSetpoint s;
  // No `active`: the ROS message has none, and set_target arms stamping regardless.
  // Clamping is core's job -- set_target is the ONE place the [0,1] range is enforced
  // for every caller, including unvalidated data off a socket.
  s.command.position = m.position;
  s.command.speed    = m.speed;
  s.command.force    = m.force;
  std::copy(m.token.begin(), m.token.end(), s.token.begin());
  return s;
}

kinova_arm_interfaces::msg::GripperState to_gripper_state_msg(
    const kinova::interface::GripperState& g) {
  kinova_arm_interfaces::msg::GripperState m;
  // g.stamp_s is deliberately NOT used: it is QUERY time, not sample time. The caller
  // stamps with the message it is publishing alongside.
  m.position = g.position;
  m.effort   = g.effort;
  m.current  = g.current;
  m.present  = g.present;
  return m;
}
```

Add `#include <algorithm>` at the top of the file if not already present.

- [ ] **Step 5: Run the tests to verify they pass**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker run --rm kinova-arm-ros2:humble bash -lc "cd /ros2_ws && colcon build --packages-select kinova_arm_ros2 >/dev/null 2>&1 && colcon test --packages-select kinova_arm_ros2 >/dev/null 2>&1; colcon test-result --verbose 2>&1 | tail -5"'
```
Expected: `0 errors, 0 failures`.

- [ ] **Step 6: Commit**

```bash
git add kinova_arm_ros2/include/kinova_arm_ros2/message_mapping.h kinova_arm_ros2/src/message_mapping.cpp kinova_arm_ros2/test/message_mapping_test.cpp
git commit -m "feat(ros2): map gripper setpoints and state; normalized to knuckle radians"
```

---

### Task 3: GripperServer

**Files:**
- Create: `kinova_arm_ros2/include/kinova_arm_ros2/gripper_server.h`
- Create: `kinova_arm_ros2/src/gripper_server.cpp`
- Create: `kinova_arm_ros2/test/fake_gripper_sink.h`
- Create: `kinova_arm_ros2/test/gripper_server_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt` (add `src/gripper_server.cpp` to the `ros2_backend` library sources; register `gripper_server_test`)

**Interfaces:**
- Consumes: `to_gripper_setpoint`, `to_gripper_state_msg` (Task 2); `kinova::interface::GripperSink`.
- Produces: `class GripperServer { GripperServer(rclcpp::Node::SharedPtr, kinova::interface::GripperSink&, bool expect_gripper); void publish_state(const builtin_interfaces::msg::Time& stamp); };` and the `kinova_arm_node: Gripper` REP 107 task.

- [ ] **Step 1: Write the fake sink**

Create `kinova_arm_ros2/test/fake_gripper_sink.h`:

```cpp
#pragma once
#include <mutex>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter's GripperSink side. Records what GripperServer delegates and
// lets a test dictate what on_query_gripper reports -- the same shape as
// FakeStreamSink, and for the same reason: no Supervisor, no URDF, no threads.
struct FakeGripperSink : public kinova::interface::GripperSink {
  mutable std::mutex m;
  std::vector<kinova::interface::GripperSetpoint> setpoints;
  kinova::interface::GripperState state{};

  void on_gripper_setpoint(const kinova::interface::GripperSetpoint& s) override {
    std::lock_guard<std::mutex> l(m);
    setpoints.push_back(s);
  }
  kinova::interface::GripperState on_query_gripper() override {
    std::lock_guard<std::mutex> l(m);
    return state;
  }
  std::size_t count() const { std::lock_guard<std::mutex> l(m); return setpoints.size(); }
};
```

- [ ] **Step 2: Write the failing test**

Create `kinova_arm_ros2/test/gripper_server_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "kinova_arm_ros2/gripper_server.h"
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"
#include "fake_gripper_sink.h"

using namespace std::chrono_literals;

class GripperServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("gripper_server_test");
  }
  void spin_for(std::chrono::milliseconds d) {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(2ms);
    }
  }
  rclcpp::Node::SharedPtr node_;
};

TEST_F(GripperServerTest, ASetpointReachesTheSinkWithAllThreeFields) {
  FakeGripperSink sink;
  kinova_arm_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  auto pub = node_->create_publisher<kinova_arm_interfaces::msg::GripperSetpoint>(
      "/setpoint/gripper", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  spin_for(200ms);   // let discovery settle

  kinova_arm_interfaces::msg::GripperSetpoint m;
  m.position = 0.6f; m.speed = 0.4f; m.force = 0.2f; m.token[0] = 3;
  for (int i = 0; i < 20 && sink.count() == 0; ++i) { pub->publish(m); spin_for(20ms); }

  ASSERT_GE(sink.count(), 1u);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.position, 0.6f);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.speed, 0.4f);
  EXPECT_FLOAT_EQ(sink.setpoints.back().command.force, 0.2f);
  EXPECT_EQ(sink.setpoints.back().token[0], 3);
}

TEST_F(GripperServerTest, PublishStateReportsWhatTheSinkSays) {
  FakeGripperSink sink;
  sink.state.position = 0.3f; sink.state.effort = 0.05f;
  sink.state.current = 0.05f; sink.state.present = true;
  kinova_arm_ros2::GripperServer server(node_, sink, /*expect_gripper=*/true);

  kinova_arm_interfaces::msg::GripperState got;
  bool seen = false;
  auto sub = node_->create_subscription<kinova_arm_interfaces::msg::GripperState>(
      "/gripper_state", rclcpp::SensorDataQoS(),
      [&](kinova_arm_interfaces::msg::GripperState::SharedPtr msg) { got = *msg; seen = true; });
  spin_for(200ms);

  for (int i = 0; i < 20 && !seen; ++i) {
    server.publish_state(node_->now());
    spin_for(20ms);
  }
  ASSERT_TRUE(seen);
  EXPECT_FLOAT_EQ(got.position, 0.3f);
  EXPECT_FLOAT_EQ(got.effort, 0.05f);
  EXPECT_TRUE(got.present);
}
```

- [ ] **Step 3: Run to verify it fails**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker run --rm kinova-arm-ros2:humble bash -lc "cd /ros2_ws && colcon build --packages-select kinova_arm_ros2 2>&1 | tail -10"'
```
Expected: FAIL — `kinova_arm_ros2/gripper_server.h: No such file or directory`.

- [ ] **Step 4: Write the header**

Create `kinova_arm_ros2/include/kinova_arm_ros2/gripper_server.h`:

```cpp
// kinova_arm_ros2/include/kinova_arm_ros2/gripper_server.h
#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "kinova_arm_interfaces/msg/gripper_setpoint.hpp"
#include "kinova_arm_interfaces/msg/gripper_state.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

// The ROS face of core's GripperSink: /setpoint/gripper in, /gripper_state out.
//
// Holds a GripperSink& and NOTHING else from core, so it unit-tests against a fake with
// no robot -- the same reason StreamServer holds only a StreamSink&.
//
// There is NO session here, unlike StreamServer. Arbiter::on_gripper_setpoint gates on
// admit(token) alone, so holding the arm's token is the whole prerequisite: the gripper
// rides the ARM's token by core's spec decision, one physical machine one holder.
class GripperServer {
 public:
  using GripperSetpointMsg = kinova_arm_interfaces::msg::GripperSetpoint;
  using GripperStateMsg    = kinova_arm_interfaces::msg::GripperState;

  // expect_gripper: "expected" cannot be inferred from the node's own model -- it loads
  // the FROZEN 7-DOF URDF, where the Robotiq joints are type="fixed", so its model never
  // has a gripper regardless of the hardware. A robot genuinely built without one sets
  // this false and the diagnostics task reports OK instead of WARN.
  GripperServer(rclcpp::Node::SharedPtr node, kinova::interface::GripperSink& sink,
                bool expect_gripper);

  // Pulls on_query_gripper() and publishes it. The caller supplies the stamp;
  // GripperState::stamp_s is QUERY time, not sample time, and is deliberately discarded.
  void publish_state(const builtin_interfaces::msg::Time& stamp);

 private:
  void on_setpoint(const GripperSetpointMsg::SharedPtr m);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::GripperSink& sink_;
  bool expect_gripper_;
  rclcpp::Subscription<GripperSetpointMsg>::SharedPtr sp_sub_;
  rclcpp::Publisher<GripperStateMsg>::SharedPtr state_pub_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 5: Write the implementation**

Create `kinova_arm_ros2/src/gripper_server.cpp`:

```cpp
#include "kinova_arm_ros2/gripper_server.h"
#include "kinova_arm_ros2/message_mapping.h"
namespace kinova_arm_ros2 {

GripperServer::GripperServer(rclcpp::Node::SharedPtr node,
                             kinova::interface::GripperSink& sink,
                             bool expect_gripper)
    : node_(std::move(node)), sink_(sink), expect_gripper_(expect_gripper) {
  // Best-effort, KeepLast(1): identical to the six arm setpoint topics, because core's
  // semantics are identical -- setpoints are absolute and latest-wins, so dropping an
  // intermediate one is correct rather than a loss.
  const auto sp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  sp_sub_ = node_->create_subscription<GripperSetpointMsg>(
      "/setpoint/gripper", sp_qos,
      std::bind(&GripperServer::on_setpoint, this, std::placeholders::_1));
  state_pub_ = node_->create_publisher<GripperStateMsg>("gripper_state",
                                                        rclcpp::SensorDataQoS());
  // Its own Updater, exactly as ArbitrationServer and Ros2Backend each own one.
  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Gripper", this, &GripperServer::diagnostics);
}

void GripperServer::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  const auto g = sink_.on_query_gripper();
  if (expect_gripper_ && !g.present) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "no gripper reported, but expect_gripper is true");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 g.present ? "gripper present" : "no gripper (expected)");
  }
  stat.add("present", g.present);
  stat.add("position", g.position);
  stat.add("effort_fraction", g.effort);
  stat.add("current_a", g.current);
}

void GripperServer::on_setpoint(const GripperSetpointMsg::SharedPtr m) {
  // Straight through to the sink, which is the ARBITER: that is what makes the token
  // load-bearing. A bad token is counted by the Arbiter, not refused here.
  sink_.on_gripper_setpoint(to_gripper_setpoint(*m));
}

void GripperServer::publish_state(const builtin_interfaces::msg::Time& stamp) {
  auto msg = to_gripper_state_msg(sink_.on_query_gripper());
  msg.header.stamp = stamp;
  state_pub_->publish(msg);
}
}  // namespace kinova_arm_ros2
```

- [ ] **Step 6: Register the source and the test in CMakeLists.txt**

In `kinova_arm_ros2/CMakeLists.txt`, add a `gripper_server` library. This repo uses ONE
LIBRARY PER SOURCE FILE (`message_mapping`, `goal_router`, `ros2_backend`,
`arbitration_server`, `stream_server` are each their own `add_library`) — there is no
combined library to append to. It must link `message_mapping`, because
`to_gripper_setpoint`/`to_gripper_state_msg` live there; `ros2_backend` links it the same
way at line 48. Omitting that link fails at the link step, not at compile:
`undefined reference to kinova_arm_ros2::to_gripper_setpoint(...)`.

Then add a test target next to the existing `stream_server_test` registration, following its
exact form:

```cmake
add_library(gripper_server src/gripper_server.cpp)
target_link_libraries(gripper_server message_mapping kinova_lowlevel::kinova_lowlevel)

ament_add_gtest(gripper_server_test test/gripper_server_test.cpp)
target_link_libraries(gripper_server_test gripper_server)
ament_target_dependencies(gripper_server_test rclcpp kinova_arm_interfaces)
target_include_directories(gripper_server_test PRIVATE test)
```

Match the surrounding style if it differs; copy how `stream_server_test` is declared.

- [ ] **Step 7: Run the tests to verify they pass**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_arm_ros2 && make build >/dev/null 2>&1; docker run --rm --network host --ipc host kinova-arm-ros2:humble bash -lc "cd /ros2_ws && colcon test --packages-select kinova_arm_ros2 >/dev/null 2>&1; colcon test-result --verbose 2>&1 | tail -6"'
```
Expected: `0 errors, 0 failures`, and the count has grown by the two new tests.

- [ ] **Step 8: Commit**

```bash
git add kinova_arm_ros2/include/kinova_arm_ros2/gripper_server.h kinova_arm_ros2/src/gripper_server.cpp kinova_arm_ros2/test/fake_gripper_sink.h kinova_arm_ros2/test/gripper_server_test.cpp kinova_arm_ros2/CMakeLists.txt
git commit -m "feat(ros2): GripperServer -- /setpoint/gripper and /gripper_state"
```

---

### Task 4: The knuckle joint in /joint_states

**Files:**
- Modify: `kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h:24` (add the setter next to `set_command_sink`) and the private members
- Modify: `kinova_arm_ros2/src/ros2_backend.cpp:91-103` (`publish_state`)
- Test: `kinova_arm_ros2/test/gripper_server_test.cpp` (append)

**Interfaces:**
- Consumes: `gripper_to_knuckle_rad` (Task 2); `kinova::interface::GripperSink`.
- Produces: `void Ros2Backend::set_gripper_sink(kinova::interface::GripperSink* sink);` — when non-null, `publish_state` appends one joint.

- [ ] **Step 1: Write the failing test**

Append to `kinova_arm_ros2/test/gripper_server_test.cpp`:

```cpp
#include "kinova_arm_ros2/ros2_backend.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>

// The joint is published even when present == false. Omitting it would leave
// robot_state_publisher without a value for an independent movable joint, and RSP then
// emits NO TF for the ENTIRE robot -- a missing gripper would break the arm's TF.
TEST_F(GripperServerTest, KnuckleJointIsPublishedEvenWhenAbsent) {
  FakeGripperSink sink;
  sink.state.position = 0.5f; sink.state.present = false;
  auto backend = std::make_shared<kinova_arm_ros2::Ros2Backend>(node_);
  backend->set_gripper_sink(&sink);

  sensor_msgs::msg::JointState got;
  bool seen = false;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::SharedPtr m) { got = *m; seen = true; });
  spin_for(200ms);

  kinova::interface::ArmState s;
  for (int i = 0; i < 20 && !seen; ++i) { backend->publish_state(s); spin_for(20ms); }

  ASSERT_TRUE(seen);
  ASSERT_EQ(got.name.size(), 8u);
  EXPECT_EQ(got.name[7], "robotiq_85_left_knuckle_joint");
  EXPECT_DOUBLE_EQ(got.position[7], 0.4);          // 0.5 * 0.8
  EXPECT_TRUE(std::isnan(got.velocity[7]));        // core removed the field entirely
  EXPECT_TRUE(std::isnan(got.effort[7]));          // 0..1 fraction, not N*m
}

// Without a sink wired, the message is exactly what it was before this tier.
TEST_F(GripperServerTest, NoGripperSinkMeansSevenJointsAsBefore) {
  auto backend = std::make_shared<kinova_arm_ros2::Ros2Backend>(node_);
  sensor_msgs::msg::JointState got;
  bool seen = false;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::SharedPtr m) { got = *m; seen = true; });
  spin_for(200ms);

  kinova::interface::ArmState s;
  for (int i = 0; i < 20 && !seen; ++i) { backend->publish_state(s); spin_for(20ms); }
  ASSERT_TRUE(seen);
  EXPECT_EQ(got.name.size(), 7u);
}
```

- [ ] **Step 2: Run to verify it fails**

Run the colcon build command from Task 3 Step 3.
Expected: FAIL — `'class kinova_arm_ros2::Ros2Backend' has no member named 'set_gripper_sink'`.

- [ ] **Step 3: Add the setter and member to ros2_backend.h**

Next to `set_command_sink` (line 24):

```cpp
  // Null until bringup wires it -- and null is a real configuration, not an omission:
  // a robot with no gripper publishes seven joints exactly as before this tier.
  void set_gripper_sink(kinova::interface::GripperSink* sink) { gripper_ = sink; }
```

And in the private section, next to `sink_` (line 54):

```cpp
  kinova::interface::GripperSink* gripper_ = nullptr;
```

- [ ] **Step 4: Append the joint in publish_state**

In `kinova_arm_ros2/src/ros2_backend.cpp`, after the `for` loop that fills the seven arm joints (line 102) and BEFORE `state_pub_->publish(msg)` (line 103), insert:

```cpp
  if (gripper_ != nullptr) {
    // ONE joint: the 2F-85 is underactuated -- one revolute DOF and five <mimic> joints
    // at +/-1, which robot_state_publisher derives itself (verified 2026-09-03).
    // Publishing the dependents too would duplicate what the model already states.
    //
    // Published REGARDLESS of `present`: omitting an independent movable joint makes RSP
    // emit no TF for the whole robot, so a missing gripper would break the ARM's TF.
    // /gripper_state carries the truth about presence instead.
    //
    // This is a second snap_ load inside core, so the value can be up to one 1 kHz
    // feedback frame newer than `s`. Irrelevant for TF; noted because the conformance
    // suite's same-pump-tick invariant covers joint_states and ee_state, not this column.
    const auto g = gripper_->on_query_gripper();
    msg.name.push_back("robotiq_85_left_knuckle_joint");
    msg.position.push_back(gripper_to_knuckle_rad(g.position));
    // NaN, never 0: sensor_msgs' convention for "no measurement". Zero would be
    // indistinguishable from "not moving" / "no load". Core has no gripper velocity at
    // all (it removed the field), and its effort is a 0..1 current fraction, not N*m.
    msg.velocity.push_back(std::numeric_limits<double>::quiet_NaN());
    msg.effort.push_back(std::numeric_limits<double>::quiet_NaN());
  }
```

Add `#include <limits>` and `#include "kinova_arm_ros2/message_mapping.h"` at the top of the file if not already present.

- [ ] **Step 5: Run the tests to verify they pass**

Run the build-and-test command from Task 3 Step 7.
Expected: `0 errors, 0 failures`.

- [ ] **Step 6: Commit**

```bash
git add kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h kinova_arm_ros2/src/ros2_backend.cpp kinova_arm_ros2/test/gripper_server_test.cpp
git commit -m "feat(ros2): publish the gripper's actuated joint in /joint_states"
```

---

### Task 5: Wire it up in bringup_node

**Files:**
- Modify: `kinova_arm_ros2/src/bringup_node.cpp:94` (decorator chain), `:160-172` (deps + servers), and the diagnostics setup

**Interfaces:**
- Consumes: `GripperServer` (Task 3), `Ros2Backend::set_gripper_sink` (Task 4).
- Produces: a running node where gripper setpoints reach the hardware.

- [ ] **Step 1: Insert GripperController into the decorator chain**

Replace line 94:

```cpp
  Seqlock<JointFeedback> snap; FeedbackTap tap(*base, snap);
```

with:

```cpp
  Seqlock<JointFeedback> snap;
  // Decorates Transport to stamp the gripper field into every outgoing frame. NOT a
  // ControlMode: modes are mutually exclusive, so making the gripper one would mean
  // giving up arm control to move it. Order follows core's own idiom in
  // apps/teleop_socket_server.cpp -- GripperController first, then FeedbackTap.
  GripperController grip(*base);
  FeedbackTap tap(grip, snap);
```

Add `#include "kinova_lowlevel/gripper_controller.h"` to the includes.

- [ ] **Step 2: Stop passing a null grip**

Replace the `deps.grip` line added by the SupervisorDeps adoption (currently absent — `grip` is left defaulted to `nullptr`) by adding, next to `deps.action = &router;`:

```cpp
  deps.grip = &grip;                  // was nullptr: every setpoint was silently dropped
```

Delete the comment block above `interface::SupervisorDeps deps;` that explains why `grip` is null, and replace it with:

```cpp
  // Core takes a SupervisorDeps aggregate rather than positional arguments, so the
  // mode/port wiring reads by name.
```

- [ ] **Step 3: Construct GripperServer and wire the backend**

After the `StreamServer stream_server(node, arb);` line, add:

```cpp
  // Wired to the Arbiter, exactly like StreamServer: that is what makes the gripper's
  // token load-bearing. There is no session to open -- Arbiter::on_gripper_setpoint
  // gates on admit(token) alone.
  kinova_arm_ros2::GripperServer gripper_server(node, arb, expect_gripper);
  backend->set_gripper_sink(&arb);
```

Add `#include "kinova_arm_ros2/gripper_server.h"` to the includes.

- [ ] **Step 4: Declare expect_gripper**

Next to the `estop_clear_max_age_s` parameter declaration, add:

```cpp
  // "Expected" cannot be inferred from the node's own model: it loads the FROZEN 7-DOF
  // URDF, in which the Robotiq joints are type="fixed", so its model never has a gripper
  // regardless of the hardware. A robot genuinely built without one sets this false.
  const bool expect_gripper = node->declare_parameter("expect_gripper", true);
```

The `Gripper` diagnostics task itself lives in `GripperServer` (Task 3), which owns its own
`Updater` exactly as `ArbitrationServer` and `Ros2Backend` each own one. Nothing to register
here.

- [ ] **Step 5: Publish gripper state on the pump tick**

`GripperServer::publish_state` needs calling. In `Ros2Backend::publish_state` the JointState stamp is `msg.header.stamp`; the simplest wiring that shares it is a 20 Hz timer in bringup, which is enough for a gripper and keeps the pump thread free of a second publisher:

```cpp
  auto gripper_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      [&gripper_server, node]() { gripper_server.publish_state(node->now()); });
```

- [ ] **Step 6: Build and run the node in sim**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_arm_ros2 && make build 2>&1 | tail -4'
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker rm -f kg >/dev/null 2>&1; docker run -d --name kg --network host --ipc host \
     -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp kinova-arm-ros2:humble \
     /ros2_ws/install/kinova_arm_ros2/lib/kinova_arm_ros2/kinova_arm_node --sim \
     --urdf /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf >/dev/null; sleep 6; \
   docker exec kg /ros_entrypoint.sh timeout 8 ros2 topic echo --once --qos-reliability best_effort /joint_states | head -12; \
   docker exec kg /ros_entrypoint.sh timeout 8 ros2 topic echo --once --qos-reliability best_effort /gripper_state; \
   docker rm -f kg >/dev/null'
```
Expected: `/joint_states` lists 8 names ending in `robotiq_85_left_knuckle_joint`, and `/gripper_state` publishes with `present: false` (SimTransport reports no gripper).

- [ ] **Step 7: Commit**

```bash
git add kinova_arm_ros2/src/bringup_node.cpp
git commit -m "feat(ros2): wire the GripperController; gripper setpoints reach the arm"
```

---

### Task 6: Make the articulated model the default

**Files:**
- Modify: `kinova_arm_description/launch/description.launch.py:7,15,49`
- Modify: `kinova_arm_description/launch/bringup.launch.py:41`

**Interfaces:**
- Consumes: the joint published in Task 4.
- Produces: `articulated` defaults to `true`; RSP renders a moving gripper.

- [ ] **Step 1: Flip the default and correct the docstrings**

In `description.launch.py`, change the declaration at line 49 from `default_value="false"` to `default_value="true"`.

Replace the module docstring's claim (lines ~7 and ~15) with:

```
robot_state_publisher DOES derive <mimic> joints -- verified 2026-09-03 with a two-joint
URDF: publishing only the driver joint moved the mimic link by exactly -0.6 rad for a
+0.6 rad drive. The articulated model has 13 movable joints but only EIGHT independent
ones (seven arm plus robotiq_85_left_knuckle_joint); RSP derives the other five.

The driver publishes that knuckle joint as of the gripper tier, so `articulated` now
defaults to true. It is safe on a gripper-less build: the joint is not in the model and
RSP ignores joint states for joints it does not know.
```

In `bringup.launch.py` line 41, replace the `articulated` argument description:

```python
            description="13-DOF model with a moving gripper; needs the driver to publish "
                        "robotiq_85_left_knuckle_joint (the gripper tier does)"
```

- [ ] **Step 2: Verify TF for the fingers actually moves**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash; \
   cd /home/abra/kinova_arm_ros2/kinova_arm_description && timeout 60 python3 -m pytest test/test_tf_updates.py -q 2>&1 | tail -5'
```
Expected: the existing TF test still passes (the arm joints are unaffected).

- [ ] **Step 3: Commit**

```bash
git add kinova_arm_description/launch/description.launch.py kinova_arm_description/launch/bringup.launch.py
git commit -m "feat(description): articulate the gripper by default"
```

---

### Task 7: Conformance section

**Files:**
- Create: `kinova_arm_ros2/test/conformance/checks_gripper.py`
- Modify: `kinova_arm_ros2/test/conformance/harness.py` (register the `gripper_state` topic)
- Modify: `kinova_arm_ros2/test/conformance/run_conformance.py:34` (`SECTION_ORDER`)
- Modify: `kinova_arm_ros2/test/conformance/README.md` (coverage table)

**Interfaces:**
- Consumes: `harness.Ctx`, `harness.tok`, `REGISTRY`, `PASS`, `FAIL`, `Result`.
- Produces: a `gripper` section.

- [ ] **Step 1: Write the checks**

Create `kinova_arm_ros2/test/conformance/checks_gripper.py`:

```python
"""The gripper tier.

Ordered AFTER motion for the same reason motion is last: these command hardware. The
gripper commands are small and slow, and open/close on empty air is safe -- but it is
still actuation, so the e-stop must already be proven in this session.
"""
import time

from kinova_arm_interfaces.msg import GripperSetpoint, GripperState
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from harness import FAIL, PASS, REGISTRY, SKIP, ZERO_TOKEN, Result, tok

SEC = "gripper"
SP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
KNUCKLE = "robotiq_85_left_knuckle_joint"


@REGISTRY.add(SEC, "/gripper_state publishes and is self-consistent")
def check_gripper_state(ctx):
    g = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if g is None:
        return Result("", FAIL, "no /gripper_state")
    if not (0.0 <= g.position <= 1.0):
        return Result("", FAIL, f"position {g.position} outside [0,1]")
    if not (0.0 <= g.effort <= 1.0):
        return Result("", FAIL, f"effort {g.effort} outside [0,1] -- it is a fraction")
    return Result("", PASS, f"present={g.present} position={g.position:.3f} "
                            f"effort={g.effort:.3f} current={g.current:.3f}A")


@REGISTRY.add(SEC, "the actuated joint appears in /joint_states, within its URDF limits")
def check_knuckle_in_joint_states(ctx):
    js = ctx.latest("joint_states", timeout=6.0, fresh=True)
    if js is None or KNUCKLE not in js.name:
        return Result("", FAIL, f"{KNUCKLE} missing from /joint_states")
    i = list(js.name).index(KNUCKLE)
    q = js.position[i]
    if not (0.0 <= q <= 0.8):
        return Result("", FAIL, f"{KNUCKLE}={q} outside the URDF limit [0, 0.8]")
    # Only the actuated joint: RSP derives the five mimics.
    mimics = [n for n in js.name if "robotiq" in n and n != KNUCKLE]
    if mimics:
        return Result("", FAIL, f"mimic joints published too: {mimics}")
    return Result("", PASS, f"{KNUCKLE}={q:.4f} rad, no mimics published")


@REGISTRY.add(SEC, "enforced: an untokened gripper setpoint is refused",
              needs_motion=True, needs_mode="enforced")
def check_untokened_refused(ctx):
    ctx.revoke("conformance: ensuring no owner")
    before = ctx.control_status().rejected_count
    pub = ctx.n.create_publisher(GripperSetpoint, "/setpoint/gripper", SP_QOS)
    ctx.spin(0.3)
    for _ in range(20):
        m = GripperSetpoint()
        m.position, m.speed, m.force = 0.0, 0.5, 0.5
        m.token = tok(ZERO_TOKEN)
        pub.publish(m)
        ctx.spin(0.02)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after <= before:
        return Result("", FAIL, "an untokened gripper setpoint was not counted as "
                                "rejected -- the Arbiter admitted it")
    return Result("", PASS, f"refused, rejected_count {before} -> {after}")


@REGISTRY.add(SEC, "enforced: a tokened setpoint moves the gripper",
              needs_motion=True, needs_mode="enforced")
def check_tokened_moves(ctx):
    token = ctx.acquire("conformance-gripper").token
    g0 = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if g0 is None or not g0.present:
        # SKIP, never PASS. The README's principle: a skip is reported, never silently
        # counted as a pass. SKIP is a first-class status the runner counts separately.
        return Result("", SKIP, "no gripper attached (present=false); nothing to command")
    pub = ctx.n.create_publisher(GripperSetpoint, "/setpoint/gripper", SP_QOS)
    ctx.spin(0.3)
    target = 0.3 if g0.position < 0.15 else 0.0
    end = time.time() + 4.0
    while time.time() < end:
        m = GripperSetpoint()
        m.position, m.speed, m.force = target, 0.3, 0.3
        m.token = tok(token)
        pub.publish(m)
        ctx.spin(0.02)
    g1 = ctx.latest("gripper_state", timeout=6.0, fresh=True)
    if abs(g1.position - g0.position) < 0.02:
        return Result("", FAIL, f"gripper did not move: {g0.position:.3f} -> "
                                f"{g1.position:.3f} commanding {target}")
    return Result("", PASS, f"moved {g0.position:.3f} -> {g1.position:.3f}")
```

- [ ] **Step 2: Register the `gripper_state` topic in the harness**

`Ctx._spec` keys topics WITHOUT a leading slash, and `latest()` returns `None` for any
topic that was never registered. In `harness.py`, add the import and one `self.sub(...)`
line beside the existing three in `Ctx.__init__`:

```python
from kinova_arm_interfaces.msg import GripperState
...
        self.sub(GripperState, "gripper_state", SENSOR_QOS)
```

- [ ] **Step 3: Register the section**

In `run_conformance.py`, change `SECTION_ORDER` (line 34) to:

```python
SECTION_ORDER = ["state", "arbitration", "streaming", "motion", "gripper"]
```

and add the import next to the others:

```python
import checks_gripper     # noqa: F401
```

- [ ] **Step 4: Run in sim**

Run:
```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'docker rm -f kg >/dev/null 2>&1; docker run -d --name kg --network host --ipc host \
     -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --cap-add SYS_NICE kinova-arm-ros2:humble \
     /ros2_ws/install/kinova_arm_ros2/lib/kinova_arm_ros2/kinova_arm_node --sim \
     --urdf /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf \
     --ros-args -p arbitration_mode:=enforced >/dev/null; sleep 7; \
   docker exec -w /ros2_ws/src/kinova_arm_ros2/kinova_arm_ros2/test/conformance kg \
     /ros_entrypoint.sh python3 -u run_conformance.py 2>&1 | tail -20; \
   docker rm -f kg >/dev/null'
```
Expected: all sections pass; the gripper section's "tokened setpoint moves" check reports SKIPPED-INLINE in sim if `present` is false.

- [ ] **Step 5: Update the conformance README**

Add a row to the coverage table:

```markdown
| `gripper` | `/gripper_state` plausible, the actuated joint in `/joint_states` within its URDF limits and **no mimics published**, an untokened setpoint counted as rejected, a tokened one moving the gripper |
```

- [ ] **Step 6: Commit**

```bash
git add kinova_arm_ros2/test/conformance/checks_gripper.py kinova_arm_ros2/test/conformance/harness.py kinova_arm_ros2/test/conformance/run_conformance.py kinova_arm_ros2/test/conformance/README.md
git commit -m "test(ros2): conformance section for the gripper tier"
```

---

## Final verification

- [ ] **Full build against plain main, all tests, conformance in sim**

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- bash -lc \
  'cd /home/abra/kinova_arm_ros2 && make build 2>&1 | tail -3'
```
Expected: `Summary: 4 packages finished`, no errors.

Then the colcon test and conformance commands from Task 3 Step 7 and Task 7 Step 3.
Expected: `0 errors, 0 failures`; conformance all-pass.

- [ ] **On-arm, attended** — per `docs/on-robot-runbook.md`. Open and close at low `force`, confirm the effort spike-then-settle signature on `/gripper_state`, and that the finger TF tracks in RViz. Record the run in the runbook.
