# ROS 2 Streaming Tier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose core's `StreamSink` — session open/close plus setpoint delivery — as ROS
services and topics, so teleop and reactive control can drive the arm at rate.

**Architecture:** `StreamServer` mirrors `ArbitrationServer`: it holds a `StreamSink&` and
nothing else from core, so it unit-tests against a fake with no robot. It is wired to the
**Arbiter**, which is how setpoint tokens get checked. A client names a *controller*; the
driver replies with the *channels* (topics) to publish on.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `rosidl`, GoogleTest via `ament_cmake_gtest`,
`kinova_lowlevel` (core).

**Spec:** `docs/superpowers/specs/2026-08-29-ros2-streaming-tier-design.md`

## Global Constraints

- **Core ref:** `feat/stream-status-port` (core PR #31 on top of #29). `on_query_stream()`
  is required; plain `feat/streaming-tier` will not compile this.
- **Setpoint QoS is best-effort, `KeepLast(1)`** — dictated by core's latest-wins
  semantics, not chosen. Reliable-with-queue would deliver stale setpoints late.
- **`/stream_status` QoS:** reliable, `transient_local`, depth 1, published on change.
- **Tokens are per message.** `uint8[16]` is `std::array<uint8_t,16>` is
  `kinova::interface::Token` — assign directly, never memcpy.
- **A controller with more than one channel cannot be opened** — core admits one
  `SetpointKind` per session. The rule applies to the *registry row*, not to client input.
- **Four callback groups:** e-stop (existing, must never wait), session (`open_stream`
  blocks 250 ms on the mode settle), setpoints (`MutuallyExclusive`), default.
- **Build/test:** `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2`, then
  `ssh abra "/tmp/rtest.sh"`. **Use `--packages-up-to`, not `--packages-select`** — the
  latter will not rebuild `kinova_lowlevel` and you will link a stale install. After any
  rename, `rm -rf build/kinova_gen3_ros2 install/kinova_gen3_ros2` on abra first, or stale
  test binaries will report passes that are not real.

---

### Task 1: Interface definitions

**Files:**
- Create: `kinova_gen3_interfaces/msg/{JointSetpoint,PoseSetpoint,TwistSetpoint,WrenchSetpoint}.msg`
- Create: `kinova_gen3_interfaces/msg/{StreamStatus,ControllerCapability}.msg`
- Create: `kinova_gen3_interfaces/srv/{OpenStream,CloseStream,ListControllers}.srv`
- Modify: `kinova_gen3_interfaces/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: `kinova_gen3_interfaces::msg::{JointSetpoint, PoseSetpoint, TwistSetpoint,
  WrenchSetpoint, StreamStatus, ControllerCapability}` and
  `srv::{OpenStream, CloseStream, ListControllers}`. Headers follow the snake_case rule
  confirmed in the arbitration tier (`EStop` → `e_stop.hpp`), so expect
  `joint_setpoint.hpp`, `open_stream.hpp`, etc.

- [ ] **Step 1: Write the four setpoint messages**

`msg/JointSetpoint.msg`:
```
# One shape, three meanings -- the TOPIC says which: rad on /setpoint/joint_position,
# rad/s on /setpoint/joint_velocity, N*m on /setpoint/joint_torque. Mirrors core's
# JointSetpoint, where the METHOD disambiguates for the same reason.
#
# Absolute, never a delta. Re-sending the same value is a no-op, and dropping an
# intermediate setpoint is correct -- which is why these topics are best-effort.
float64[7] values
uint8[16]  token     # from /acquire_control; validates the sender, every message
```

`msg/PoseSetpoint.msg`:
```
# Target tool pose in the base frame.
geometry_msgs/Pose pose
uint8[16]          token
```

`msg/TwistSetpoint.msg`:
```
# Target tool twist, base frame. Core carries this as [linear; angular].
geometry_msgs/Twist twist
uint8[16]           token
```

`msg/WrenchSetpoint.msg`:
```
# Target tool wrench, base frame. NOTE: no controller can consume this yet -- core has
# no SetpointKind::kEeWrench. The topic exists so the surface is complete and clients
# can be written against it; setpoints are dropped with a throttled warning.
geometry_msgs/Wrench wrench
uint8[16]            token
```

- [ ] **Step 2: Write the status and capability messages**

`msg/StreamStatus.msg`:
```
# What the streaming tier is actually doing. `open`, `timeout_s` and `rejected_count`
# come from core via StreamSink::on_query_stream(), so they are exact rather than this
# node's guess -- a session torn down on deadline expiry shows up here immediately.
#
# `rejected_count` counts setpoints the SESSION refused (wrong channel, or closed).
# Token failures are counted by the Arbiter and appear on /control_status instead.
std_msgs/Header header
bool     open
string   controller       # "" when closed; this node's label for core's (kind, mode)
string[] channels         # topics the open controller reads
float64  timeout_s
uint64   rejected_count
```

`msg/ControllerCapability.msg`:
```
string   name         # e.g. "joint_impedance"
string[] channels     # topics under /setpoint/ this controller reads
bool     available    # computed live from core's pair_supported(), never declared
```

- [ ] **Step 3: Write the three services**

`srv/OpenStream.srv`:
```
# Open a streaming session on a controller. The (kind, control mode) pair is fixed for
# the session's lifetime -- changing what you stream means close-then-reopen, which
# re-pays the 250 ms mode settle.
#
# Call /list_controllers first: you must create your publisher and let DDS discovery
# settle BEFORE opening, or your first setpoints go nowhere and the session expires.
string    controller
float64   timeout_s     # MUST be > 0; an unbounded stream has no safe-stop
uint8[16] token
---
bool     accepted
string[] channels       # where to publish -- the driver tells you
int32    error_code     # 0, or -10 STREAM_REJECTED
string   message
```

`srv/CloseStream.srv`:
```
uint8[16] token
---
bool   closed
string message
```

`srv/ListControllers.srv`:
```
---
ControllerCapability[] controllers
```

- [ ] **Step 4: Register them in `kinova_gen3_interfaces/CMakeLists.txt`**

Add to `rosidl_generate_interfaces`, after `"srv/RevokeControl.srv"`:
```cmake
  "msg/JointSetpoint.msg"
  "msg/PoseSetpoint.msg"
  "msg/TwistSetpoint.msg"
  "msg/WrenchSetpoint.msg"
  "msg/StreamStatus.msg"
  "msg/ControllerCapability.msg"
  "srv/OpenStream.srv"
  "srv/CloseStream.srv"
  "srv/ListControllers.srv"
```
`geometry_msgs` is already in `DEPENDENCIES`; no change needed there.

- [ ] **Step 5: Build and verify**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2`
Expected: build succeeds.

```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  ros2 interface show kinova_gen3_interfaces/srv/OpenStream && \
  ros2 interface show kinova_gen3_interfaces/msg/JointSetpoint'"
```
Expected: both print.

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_interfaces/
git commit -m "feat(interfaces): setpoint messages, stream session services, controller capabilities"
```

---

### Task 2: `StreamServer` — controller registry and `/list_controllers`

**Files:**
- Create: `kinova_gen3_ros2/include/kinova_gen3_ros2/stream_server.h`
- Create: `kinova_gen3_ros2/src/stream_server.cpp`
- Create: `kinova_gen3_ros2/test/fake_stream_sink.h`
- Create: `kinova_gen3_ros2/test/stream_server_test.cpp`
- Modify: `kinova_gen3_ros2/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1's messages and services.
- Produces: `kinova_gen3_ros2::StreamServer`, constructed as
  `StreamServer(rclcpp::Node::SharedPtr, kinova::interface::StreamSink&)`, with one public
  method `void publish_status_if_changed()`. Task 6 constructs it. Also produces
  `FakeStreamSink` (test-only), used by Tasks 3–5.

- [ ] **Step 1: Write the fake sink**

`kinova_gen3_ros2/test/fake_stream_sink.h`:
```cpp
#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter's StreamSink side. Records what StreamServer delegates and
// lets a test dictate what on_query_stream reports, so status can be exercised without
// a Supervisor, a URDF or threads.
struct FakeStreamSink : public kinova::interface::StreamSink {
  mutable std::mutex m;
  std::vector<std::string> calls;
  kinova::interface::StreamOpenRequest last_open{};
  kinova::interface::Token last_token{};
  kinova::JointVec last_values = kinova::JointVec::Zero();
  kinova::interface::StreamStatus status{};
  bool accept_open = true;

  void note(const std::string& s) { std::lock_guard<std::mutex> l(m); calls.push_back(s); }
  std::vector<std::string> log() const { std::lock_guard<std::mutex> l(m); return calls; }

  kinova::interface::StreamOpenResult on_stream_open(
      const kinova::interface::StreamOpenRequest& r) override {
    note("open");
    { std::lock_guard<std::mutex> l(m); last_open = r; }
    if (!accept_open) return {false, kinova::interface::result_code::kStreamRejected,
                              "refused by fake"};
    return {true, 0, ""};
  }
  void on_stream_close(const kinova::interface::StreamCloseRequest& r) override {
    note("close");
    std::lock_guard<std::mutex> l(m); last_token = r.token;
  }
  void on_setpoint_joint_position(const kinova::interface::JointSetpoint& s) override {
    note("joint_position");
    std::lock_guard<std::mutex> l(m); last_token = s.token; last_values = s.values;
  }
  void on_setpoint_joint_velocity(const kinova::interface::JointSetpoint& s) override {
    note("joint_velocity");
    std::lock_guard<std::mutex> l(m); last_token = s.token; last_values = s.values;
  }
  void on_setpoint_joint_torque(const kinova::interface::JointSetpoint& s) override {
    note("joint_torque");
    std::lock_guard<std::mutex> l(m); last_token = s.token; last_values = s.values;
  }
  void on_setpoint_pose(const kinova::interface::PoseSetpoint& s) override {
    note("pose");
    std::lock_guard<std::mutex> l(m); last_token = s.token;
  }
  void on_setpoint_twist(const kinova::interface::TwistSetpoint& s) override {
    note("twist");
    std::lock_guard<std::mutex> l(m); last_token = s.token;
  }
  kinova::interface::StreamStatus on_query_stream() override {
    std::lock_guard<std::mutex> l(m); return status;
  }
};
```

- [ ] **Step 2: Write the failing test**

`kinova_gen3_ros2/test/stream_server_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_ros2/stream_server.h"
#include "fake_stream_sink.h"
using namespace std::chrono_literals;

namespace {
class SpinThread {
 public:
  explicit SpinThread(rclcpp::Executor& ex) : ex_(ex), t_([&ex] { ex.spin(); }) {}
  ~SpinThread() { ex_.cancel(); if (t_.joinable()) t_.join(); }
  SpinThread(const SpinThread&) = delete;
  SpinThread& operator=(const SpinThread&) = delete;
 private:
  rclcpp::Executor& ex_;
  std::thread t_;
};

kinova::interface::Token mktoken(uint8_t x) {
  kinova::interface::Token t{}; t[0] = x; return t;
}

class StreamServerTest : public ::testing::Test {
 protected:
  // The executor must NOT be a member: members are constructed before SetUp runs, and
  // building one before rclcpp::init throws "context argument is null".
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("stream_server_test");
    server_ = std::make_unique<kinova_gen3_ros2::StreamServer>(node_, sink_);
    ex_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    ex_->add_node(node_);
  }
  void TearDown() override {
    ex_.reset(); server_.reset(); node_.reset(); rclcpp::shutdown();
  }
  template <class SrvT>
  typename SrvT::Response::SharedPtr call(const std::string& name,
                                          typename SrvT::Request::SharedPtr req) {
    auto client = node_->create_client<SrvT>(name);
    SpinThread spin(*ex_);
    if (!client->wait_for_service(3s)) return nullptr;
    auto fut = client->async_send_request(req);
    if (fut.wait_for(3s) != std::future_status::ready) return nullptr;
    return fut.get();
  }
  rclcpp::Node::SharedPtr node_;
  FakeStreamSink sink_;
  std::unique_ptr<kinova_gen3_ros2::StreamServer> server_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> ex_;
};
}  // namespace

TEST_F(StreamServerTest, ListsEveryControllerWithItsChannels) {
  using Srv = kinova_gen3_interfaces::srv::ListControllers;
  auto resp = call<Srv>("list_controllers", std::make_shared<Srv::Request>());
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->controllers.size(), 7u);

  auto find = [&](const std::string& n) -> const kinova_gen3_interfaces::msg::ControllerCapability* {
    for (const auto& c : resp->controllers) if (c.name == n) return &c;
    return nullptr;
  };
  const auto* imp = find("joint_impedance");
  ASSERT_NE(imp, nullptr);
  EXPECT_TRUE(imp->available);
  ASSERT_EQ(imp->channels.size(), 1u);
  EXPECT_EQ(imp->channels[0], "joint_position");

  // Availability is computed from core's pair_supported(), so these track core rather
  // than a hand-maintained list: they light up when JointVelocityMode lands.
  const auto* vel = find("joint_velocity");
  ASSERT_NE(vel, nullptr);
  EXPECT_FALSE(vel->available);

  // Unavailable for two independent reasons: core has no kEeWrench, AND a two-channel
  // controller needs multi-channel sessions.
  const auto* cart = find("cartesian_impedance");
  ASSERT_NE(cart, nullptr);
  EXPECT_FALSE(cart->available);
  EXPECT_EQ(cart->channels.size(), 2u);
}
```

- [ ] **Step 3: Run it to verify it fails**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2`
Expected: FAIL — `kinova_gen3_ros2/stream_server.h: No such file or directory`.

- [ ] **Step 4: Write the header**

`kinova_gen3_ros2/include/kinova_gen3_ros2/stream_server.h`:
```cpp
// kinova_gen3_ros2/include/kinova_gen3_ros2/stream_server.h
#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_interfaces/msg/controller_capability.hpp"
#include "kinova_gen3_interfaces/msg/joint_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/pose_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/stream_status.hpp"
#include "kinova_gen3_interfaces/msg/twist_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/wrench_setpoint.hpp"
#include "kinova_gen3_interfaces/srv/close_stream.hpp"
#include "kinova_gen3_interfaces/srv/list_controllers.hpp"
#include "kinova_gen3_interfaces/srv/open_stream.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's StreamSink: session services, the setpoint topics, and
// /stream_status.
//
// Holds a StreamSink& and NOTHING else from core, so it unit-tests against a fake with
// no robot -- the same reason ArbitrationServer holds only an ArbitrationSink&.
//
// A client names a CONTROLLER; the driver replies with the CHANNELS to publish on. Core
// models a session as a (SetpointKind, ControlModeKind) pair, which would make 20
// combinations representable when 5 are legal; the registry below is the only place
// that collapse lives.
class StreamServer {
 public:
  using OpenStream      = kinova_gen3_interfaces::srv::OpenStream;
  using CloseStream     = kinova_gen3_interfaces::srv::CloseStream;
  using ListControllers = kinova_gen3_interfaces::srv::ListControllers;
  using StreamStatusMsg = kinova_gen3_interfaces::msg::StreamStatus;
  using JointSetpointMsg  = kinova_gen3_interfaces::msg::JointSetpoint;
  using PoseSetpointMsg   = kinova_gen3_interfaces::msg::PoseSetpoint;
  using TwistSetpointMsg  = kinova_gen3_interfaces::msg::TwistSetpoint;
  using WrenchSetpointMsg = kinova_gen3_interfaces::msg::WrenchSetpoint;

  StreamServer(rclcpp::Node::SharedPtr node, kinova::interface::StreamSink& sink);

  // Publish only if core's stream state differs from what was last sent. Called by the
  // session handlers and by a 10 Hz timer, so a teardown core did on its own (deadline
  // expiry, halt) still surfaces.
  void publish_status_if_changed();

  // One controller: a control law plus the channels it reads. `core_backed` is false
  // when core has no SetpointKind for it at all, so there is nothing to ask
  // pair_supported() about.
  struct ControllerRow {
    std::string name;
    std::vector<std::string> channels;
    bool core_backed;
    kinova::interface::SetpointKind    kind;   // meaningful iff core_backed
    kinova::interface::ControlModeKind mode;   // meaningful iff core_backed
  };
  static const std::vector<ControllerRow>& registry();
  // Computed live from core, never declared: a controller is openable only if core has
  // a kind for it, that pair is supported, and it needs just one channel (core admits
  // exactly one SetpointKind per session).
  static bool available(const ControllerRow&);

 private:
  void on_open(const std::shared_ptr<OpenStream::Request>,
               std::shared_ptr<OpenStream::Response>);
  void on_close(const std::shared_ptr<CloseStream::Request>,
                std::shared_ptr<CloseStream::Response>);
  void on_list(const std::shared_ptr<ListControllers::Request>,
               std::shared_ptr<ListControllers::Response>);

  void on_joint_position(const JointSetpointMsg::SharedPtr);
  void on_joint_velocity(const JointSetpointMsg::SharedPtr);
  void on_joint_torque(const JointSetpointMsg::SharedPtr);
  void on_pose(const PoseSetpointMsg::SharedPtr);
  void on_twist(const TwistSetpointMsg::SharedPtr);
  void on_wrench(const WrenchSetpointMsg::SharedPtr);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::StreamSink& sink_;

  rclcpp::Service<OpenStream>::SharedPtr open_srv_;
  rclcpp::Service<CloseStream>::SharedPtr close_srv_;
  rclcpp::Service<ListControllers>::SharedPtr list_srv_;
  rclcpp::Publisher<StreamStatusMsg>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr session_group_;
  rclcpp::CallbackGroup::SharedPtr setpoint_group_;
  rclcpp::Subscription<JointSetpointMsg>::SharedPtr jp_sub_, jv_sub_, jt_sub_;
  rclcpp::Subscription<PoseSetpointMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<TwistSetpointMsg>::SharedPtr twist_sub_;
  rclcpp::Subscription<WrenchSetpointMsg>::SharedPtr wrench_sub_;

  std::mutex m_;
  std::string open_controller_;              // our label for core's (kind, mode)
  std::optional<StreamStatusMsg> last_published_;
};
}  // namespace kinova_gen3_ros2
```

- [ ] **Step 5: Write the registry and `/list_controllers`**

Create `kinova_gen3_ros2/src/stream_server.cpp` with the registry, availability, the
constructor's service/publisher setup, and `on_list`. The remaining handlers arrive in
Tasks 3–5; stub them as empty bodies for now so the file compiles.

```cpp
// kinova_gen3_ros2/src/stream_server.cpp
#include "kinova_gen3_ros2/stream_server.h"
#include <chrono>
#include <functional>
#include "kinova_lowlevel/interface/streaming_session.h"   // pair_supported
namespace kinova_gen3_ros2 {
using namespace kinova::interface;
using std::placeholders::_1; using std::placeholders::_2;

const std::vector<StreamServer::ControllerRow>& StreamServer::registry() {
  // The ONLY place core's (kind, mode) pair is collapsed into a controller name.
  // cartesian_impedance is core_backed=false: core has no kEeWrench, so there is no
  // pair to ask about -- and it needs two channels besides.
  static const std::vector<ControllerRow> kRows = {
    {"joint_position",      {"joint_position"}, true,
       SetpointKind::kJointPosition, ControlModeKind::kPosition},
    {"joint_impedance",     {"joint_position"}, true,
       SetpointKind::kJointPosition, ControlModeKind::kImpedance},
    {"ee_pose_impedance",   {"pose"},           true,
       SetpointKind::kEePose,        ControlModeKind::kImpedance},
    {"joint_torque",        {"joint_torque"},   true,
       SetpointKind::kJointTorque,   ControlModeKind::kTorque},
    {"joint_velocity",      {"joint_velocity"}, true,
       SetpointKind::kJointVelocity, ControlModeKind::kVelocity},
    {"ee_twist",            {"twist"},          true,
       SetpointKind::kEeTwist,       ControlModeKind::kVelocity},
    {"cartesian_impedance", {"pose", "wrench"}, false,
       SetpointKind::kEePose,        ControlModeKind::kImpedance},
  };
  return kRows;
}

bool StreamServer::available(const ControllerRow& r) {
  // Asking core rather than declaring: when JointVelocityMode lands, these rows light
  // up with no change here.
  return r.core_backed && r.channels.size() == 1 && pair_supported(r.kind, r.mode);
}

StreamServer::StreamServer(rclcpp::Node::SharedPtr node, StreamSink& sink)
    : node_(node), sink_(sink) {
  // on_stream_open sleeps mode_settle_s (250 ms) holding the arbiter mutex, so session
  // control gets its own group -- never /estop's, and never the setpoints'.
  session_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sp_opts;
  // MutuallyExclusive: one logical writer, matching core's single-writer double buffer.
  // Separate from session_group_ so setpoints do not queue behind a 250 ms open.
  setpoint_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sp_opts.callback_group = setpoint_group_;

  open_srv_ = node_->create_service<OpenStream>(
      "open_stream", std::bind(&StreamServer::on_open, this, _1, _2),
      rmw_qos_profile_services_default, session_group_);
  close_srv_ = node_->create_service<CloseStream>(
      "close_stream", std::bind(&StreamServer::on_close, this, _1, _2),
      rmw_qos_profile_services_default, session_group_);
  list_srv_ = node_->create_service<ListControllers>(
      "list_controllers", std::bind(&StreamServer::on_list, this, _1, _2),
      rmw_qos_profile_services_default, session_group_);

  status_pub_ = node_->create_publisher<StreamStatusMsg>(
      "stream_status", rclcpp::QoS(1).reliable().transient_local());

  // Best-effort, KeepLast(1): core's setpoints are absolute and latest-wins, so a
  // dropped intermediate is correct. Reliable-with-queue would deliver stale setpoints
  // late, which is the failure the session deadline exists to catch.
  const auto sp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  jp_sub_ = node_->create_subscription<JointSetpointMsg>(
      "/setpoint/joint_position", sp_qos,
      std::bind(&StreamServer::on_joint_position, this, _1), sp_opts);
  jv_sub_ = node_->create_subscription<JointSetpointMsg>(
      "/setpoint/joint_velocity", sp_qos,
      std::bind(&StreamServer::on_joint_velocity, this, _1), sp_opts);
  jt_sub_ = node_->create_subscription<JointSetpointMsg>(
      "/setpoint/joint_torque", sp_qos,
      std::bind(&StreamServer::on_joint_torque, this, _1), sp_opts);
  pose_sub_ = node_->create_subscription<PoseSetpointMsg>(
      "/setpoint/pose", sp_qos, std::bind(&StreamServer::on_pose, this, _1), sp_opts);
  twist_sub_ = node_->create_subscription<TwistSetpointMsg>(
      "/setpoint/twist", sp_qos, std::bind(&StreamServer::on_twist, this, _1), sp_opts);
  wrench_sub_ = node_->create_subscription<WrenchSetpointMsg>(
      "/setpoint/wrench", sp_qos, std::bind(&StreamServer::on_wrench, this, _1), sp_opts);

  status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                           [this] { publish_status_if_changed(); });
  publish_status_if_changed();   // seed the latched topic
}

void StreamServer::on_list(const std::shared_ptr<ListControllers::Request>,
                           std::shared_ptr<ListControllers::Response> resp) {
  for (const auto& r : registry()) {
    kinova_gen3_interfaces::msg::ControllerCapability c;
    c.name = r.name;
    c.channels = r.channels;
    c.available = available(r);
    resp->controllers.push_back(c);
  }
}

// --- filled in by later tasks -------------------------------------------------------
void StreamServer::on_open(const std::shared_ptr<OpenStream::Request>,
                           std::shared_ptr<OpenStream::Response>) {}
void StreamServer::on_close(const std::shared_ptr<CloseStream::Request>,
                            std::shared_ptr<CloseStream::Response>) {}
void StreamServer::on_joint_position(const JointSetpointMsg::SharedPtr) {}
void StreamServer::on_joint_velocity(const JointSetpointMsg::SharedPtr) {}
void StreamServer::on_joint_torque(const JointSetpointMsg::SharedPtr) {}
void StreamServer::on_pose(const PoseSetpointMsg::SharedPtr) {}
void StreamServer::on_twist(const TwistSetpointMsg::SharedPtr) {}
void StreamServer::on_wrench(const WrenchSetpointMsg::SharedPtr) {}
void StreamServer::publish_status_if_changed() {}
}  // namespace kinova_gen3_ros2
```

- [ ] **Step 6: Add the build targets**

In `kinova_gen3_ros2/CMakeLists.txt`, after the `control_plane`/`arbitration_server`
library block:
```cmake
# The ROS face of core's StreamSink: session services, setpoint topics, /stream_status.
add_library(stream_server src/stream_server.cpp)
target_include_directories(stream_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(stream_server rclcpp kinova_gen3_interfaces geometry_msgs std_msgs)
target_link_libraries(stream_server kinova_lowlevel::kinova_lowlevel)
```
Add `stream_server` to `kinova_gen3_node`'s `target_link_libraries`, and inside
`if(BUILD_TESTING)`:
```cmake
  ament_add_gtest(stream_server_test test/stream_server_test.cpp)
  target_include_directories(stream_server_test PRIVATE test)
  target_link_libraries(stream_server_test stream_server)
  ament_target_dependencies(stream_server_test rclcpp kinova_gen3_interfaces geometry_msgs)
```

- [ ] **Step 7: Run the test**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2` then
`ssh abra "/tmp/rtest.sh stream_server_test"`
Expected: 1 test, 0 failures.

- [ ] **Step 8: Commit**

```bash
git add kinova_gen3_ros2/include/kinova_gen3_ros2/stream_server.h \
        kinova_gen3_ros2/src/stream_server.cpp \
        kinova_gen3_ros2/test/stream_server_test.cpp \
        kinova_gen3_ros2/test/fake_stream_sink.h kinova_gen3_ros2/CMakeLists.txt
git commit -m "feat(ros2): StreamServer controller registry and /list_controllers"
```

---

### Task 3: Session open and close

**Files:**
- Modify: `kinova_gen3_ros2/src/stream_server.cpp` (`on_open`, `on_close`)
- Modify: `kinova_gen3_ros2/test/stream_server_test.cpp`

**Interfaces:**
- Consumes: `StreamServer` and `FakeStreamSink` from Task 2.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST_F(StreamServerTest, OpenMapsTheControllerOntoCoresPair) {
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "joint_impedance";
  req->timeout_s = 0.1;
  req->token = mktoken(0xAB);
  auto resp = call<Srv>("open_stream", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->accepted);
  ASSERT_EQ(resp->channels.size(), 1u);
  EXPECT_EQ(resp->channels[0], "joint_position");   // the driver tells you where

  EXPECT_EQ(sink_.log().back(), "open");
  EXPECT_EQ(sink_.last_open.kind, kinova::interface::SetpointKind::kJointPosition);
  EXPECT_EQ(sink_.last_open.control_mode, kinova::interface::ControlModeKind::kImpedance);
  EXPECT_DOUBLE_EQ(sink_.last_open.timeout_s, 0.1);
  EXPECT_EQ(sink_.last_open.token, mktoken(0xAB));
}

// An unknown name must not reach core -- core would have to invent an error for
// something that is purely this layer's vocabulary.
TEST_F(StreamServerTest, UnknownControllerIsRejectedWithoutReachingCore) {
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "nonsense";
  req->timeout_s = 0.1;
  auto resp = call<Srv>("open_stream", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->accepted);
  EXPECT_EQ(resp->error_code, kinova::interface::result_code::kStreamRejected);
  EXPECT_TRUE(sink_.log().empty());
}

// cartesian_impedance is unavailable in this driver version, and core has no kind for
// it at all -- so the rejection has to originate here, not in pair_supported().
TEST_F(StreamServerTest, UnavailableControllerIsRejectedWithoutReachingCore) {
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "cartesian_impedance";
  req->timeout_s = 0.1;
  auto resp = call<Srv>("open_stream", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->accepted);
  EXPECT_TRUE(sink_.log().empty());
}

TEST_F(StreamServerTest, CoresRejectionIsRelayedVerbatim) {
  sink_.accept_open = false;
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "joint_torque";
  req->timeout_s = 0.1;
  auto resp = call<Srv>("open_stream", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->accepted);
  EXPECT_EQ(resp->message, "refused by fake");
  EXPECT_EQ(resp->error_code, kinova::interface::result_code::kStreamRejected);
}

TEST_F(StreamServerTest, CloseForwardsTheToken) {
  using Srv = kinova_gen3_interfaces::srv::CloseStream;
  auto req = std::make_shared<Srv::Request>();
  req->token = mktoken(0xCD);
  auto resp = call<Srv>("close_stream", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->closed);
  EXPECT_EQ(sink_.log().back(), "close");
  EXPECT_EQ(sink_.last_token, mktoken(0xCD));
}
```

- [ ] **Step 2: Run to verify they fail**

Run: build, then `ssh abra "/tmp/rtest.sh stream_server_test"`
Expected: FAIL — `accepted` is false / `channels` empty, because `on_open` is a stub.

- [ ] **Step 3: Implement `on_open` and `on_close`**

Replace the two stubs in `stream_server.cpp`:
```cpp
void StreamServer::on_open(const std::shared_ptr<OpenStream::Request> req,
                           std::shared_ptr<OpenStream::Response> resp) {
  const ControllerRow* row = nullptr;
  for (const auto& r : registry()) if (r.name == req->controller) { row = &r; break; }

  // Two rejections originate HERE rather than in core: an unknown name (purely this
  // layer's vocabulary) and an unavailable controller (core may have no kind for it at
  // all, so there is nothing to ask pair_supported about).
  if (!row) {
    resp->accepted = false;
    resp->error_code = result_code::kStreamRejected;
    resp->message = "unknown controller '" + req->controller +
                    "'; call /list_controllers for the available set";
    RCLCPP_WARN(node_->get_logger(), "%s", resp->message.c_str());
    return;
  }
  if (!available(*row)) {
    resp->accepted = false;
    resp->error_code = result_code::kStreamRejected;
    resp->message = "controller '" + row->name +
                    "' is not available in this driver version";
    RCLCPP_WARN(node_->get_logger(), "%s", resp->message.c_str());
    return;
  }

  StreamOpenRequest r;
  r.kind = row->kind;
  r.control_mode = row->mode;
  r.timeout_s = req->timeout_s;
  r.token = req->token;
  const StreamOpenResult res = sink_.on_stream_open(r);   // blocks the mode settle

  resp->accepted = res.accepted;
  resp->error_code = res.error_code;
  resp->message = res.message;
  if (res.accepted) {
    resp->channels = row->channels;              // the driver names the topics
    std::lock_guard<std::mutex> l(m_);
    open_controller_ = row->name;
  }
  publish_status_if_changed();
}

void StreamServer::on_close(const std::shared_ptr<CloseStream::Request> req,
                            std::shared_ptr<CloseStream::Response> resp) {
  StreamCloseRequest c;
  c.token = req->token;
  sink_.on_stream_close(c);
  { std::lock_guard<std::mutex> l(m_); open_controller_.clear(); }
  resp->closed = true;
  resp->message = "";
  publish_status_if_changed();
}
```

- [ ] **Step 4: Run the tests**

Run: build, then `ssh abra "/tmp/rtest.sh stream_server_test"`
Expected: 6 tests, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/src/stream_server.cpp kinova_gen3_ros2/test/stream_server_test.cpp
git commit -m "feat(ros2): open and close a streaming session by controller name"
```

---

### Task 4: Setpoint routing

**Files:**
- Modify: `kinova_gen3_ros2/src/stream_server.cpp` (six subscription handlers)
- Modify: `kinova_gen3_ros2/test/stream_server_test.cpp`

**Interfaces:**
- Consumes: `StreamServer`, `FakeStreamSink`.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Add a publish helper to the fixture:
```cpp
  template <class MsgT>
  void publish_setpoint(const std::string& topic, const MsgT& m) {
    auto pub = node_->create_publisher<MsgT>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    SpinThread spin(*ex_);
    for (int i = 0; i < 200 && pub->get_subscription_count() == 0; ++i)
      std::this_thread::sleep_for(10ms);
    pub->publish(m);
    std::this_thread::sleep_for(300ms);
  }
```
and the tests:
```cpp
TEST_F(StreamServerTest, JointTopicsRouteToTheirOwnSinkMethod) {
  kinova_gen3_interfaces::msg::JointSetpoint m;
  m.values = {0.1, 0, 0, 0, 0, 0, 0};
  m.token = mktoken(0xAB);

  publish_setpoint("/setpoint/joint_position", m);
  EXPECT_EQ(sink_.log().back(), "joint_position");
  EXPECT_EQ(sink_.last_token, mktoken(0xAB));      // token survives the hop
  EXPECT_NEAR(sink_.last_values[0], 0.1, 1e-12);

  publish_setpoint("/setpoint/joint_velocity", m);
  EXPECT_EQ(sink_.log().back(), "joint_velocity");

  publish_setpoint("/setpoint/joint_torque", m);
  EXPECT_EQ(sink_.log().back(), "joint_torque");
}

TEST_F(StreamServerTest, PoseAndTwistRouteToTheirOwnSinkMethod) {
  kinova_gen3_interfaces::msg::PoseSetpoint p;
  p.pose.position.x = 0.4;
  p.pose.orientation.w = 1.0;
  p.token = mktoken(0x11);
  publish_setpoint("/setpoint/pose", p);
  EXPECT_EQ(sink_.log().back(), "pose");
  EXPECT_EQ(sink_.last_token, mktoken(0x11));

  kinova_gen3_interfaces::msg::TwistSetpoint t;
  t.twist.linear.x = 0.05;
  t.token = mktoken(0x22);
  publish_setpoint("/setpoint/twist", t);
  EXPECT_EQ(sink_.log().back(), "twist");
  EXPECT_EQ(sink_.last_token, mktoken(0x22));
}

// Core has no on_setpoint_wrench, so there is nowhere to route this. The topic exists
// so the surface is complete; the message is dropped, loudly enough to diagnose.
TEST_F(StreamServerTest, WrenchIsDroppedBecauseCoreHasNoSinkForIt) {
  kinova_gen3_interfaces::msg::WrenchSetpoint w;
  w.wrench.force.z = 5.0;
  w.token = mktoken(0x33);
  publish_setpoint("/setpoint/wrench", w);
  EXPECT_TRUE(sink_.log().empty());
}
```

- [ ] **Step 2: Run to verify they fail**

Expected: FAIL — `sink_.log()` is empty for the routing tests, because the handlers are
stubs. (`WrenchIsDroppedBecauseCoreHasNoSinkForIt` will pass trivially; that is fine —
it is a guard against someone later wiring wrench to the wrong method.)

- [ ] **Step 3: Implement the six handlers**

Replace the six stubs. Add these conversion helpers to the anonymous namespace at the top
of `stream_server.cpp` — they live here rather than in `message_mapping` because only the
streaming tier converts setpoint payloads:
```cpp
namespace {
kinova::JointVec to_joint_vec(const std::array<double, 7>& a) {
  kinova::JointVec v;
  for (int i = 0; i < kinova::kNumJoints; ++i) v[i] = a[i];
  return v;
}
kinova::Pose to_pose(const geometry_msgs::msg::Pose& p) {
  kinova::Pose out;
  out.p = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  out.R = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                             p.orientation.y, p.orientation.z);
  return out;
}
kinova::Vector6 to_vector6(const geometry_msgs::msg::Twist& t) {
  kinova::Vector6 v;                      // core carries this as [linear; angular]
  v << t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z;
  return v;
}
}  // namespace
```
and the handlers:
```cpp
void StreamServer::on_joint_position(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s; s.values = to_joint_vec(m->values); s.token = m->token;
  sink_.on_setpoint_joint_position(s);
}
void StreamServer::on_joint_velocity(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s; s.values = to_joint_vec(m->values); s.token = m->token;
  sink_.on_setpoint_joint_velocity(s);
}
void StreamServer::on_joint_torque(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s; s.values = to_joint_vec(m->values); s.token = m->token;
  sink_.on_setpoint_joint_torque(s);
}
void StreamServer::on_pose(const PoseSetpointMsg::SharedPtr m) {
  PoseSetpoint s; s.pose = to_pose(m->pose); s.token = m->token;
  sink_.on_setpoint_pose(s);
}
void StreamServer::on_twist(const TwistSetpointMsg::SharedPtr m) {
  TwistSetpoint s; s.twist = to_vector6(m->twist); s.token = m->token;
  sink_.on_setpoint_twist(s);
}
void StreamServer::on_wrench(const WrenchSetpointMsg::SharedPtr) {
  // Core has no SetpointKind::kEeWrench and no on_setpoint_wrench, so there is nothing
  // to delegate to. Throttled because a client streaming wrench will send at rate.
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "/setpoint/wrench: no controller in this driver version consumes "
                       "wrench setpoints; dropping");
}
```
Add `#include "geometry_msgs/msg/pose.hpp"` and `"geometry_msgs/msg/twist.hpp"` to
`stream_server.cpp`.

- [ ] **Step 4: Run the tests**

Expected: 9 tests, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/src/stream_server.cpp kinova_gen3_ros2/test/stream_server_test.cpp
git commit -m "feat(ros2): route the six setpoint topics onto StreamSink"
```

---

### Task 5: `/stream_status`

**Files:**
- Modify: `kinova_gen3_ros2/src/stream_server.cpp` (`publish_status_if_changed`)
- Modify: `kinova_gen3_ros2/test/stream_server_test.cpp`

**Interfaces:**
- Consumes: `StreamServer`, `FakeStreamSink`, core's `StreamSink::on_query_stream()`.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Add a collector to the fixture:
```cpp
  std::vector<kinova_gen3_interfaces::msg::StreamStatus> collect_status(
      std::chrono::milliseconds dwell) {
    std::vector<kinova_gen3_interfaces::msg::StreamStatus> got;
    std::mutex gm;
    auto sub = node_->create_subscription<kinova_gen3_interfaces::msg::StreamStatus>(
        "stream_status", rclcpp::QoS(10).reliable().transient_local(),
        [&got, &gm](kinova_gen3_interfaces::msg::StreamStatus::SharedPtr m) {
          std::lock_guard<std::mutex> l(gm); got.push_back(*m);
        });
    { SpinThread spin(*ex_); std::this_thread::sleep_for(dwell); }
    std::lock_guard<std::mutex> l(gm);
    return got;
  }
```
and the tests:
```cpp
// open/timeout/rejected_count come from core, NOT from what this node remembers, so a
// session core tore down on its own still surfaces.
TEST_F(StreamServerTest, StatusReportsCoresViewNotOurs) {
  sink_.status.open = true;
  sink_.status.timeout_s = 0.25;
  sink_.status.rejected_count = 4;
  const auto got = collect_status(400ms);
  ASSERT_FALSE(got.empty());
  EXPECT_TRUE(got.back().open);
  EXPECT_DOUBLE_EQ(got.back().timeout_s, 0.25);
  EXPECT_EQ(got.back().rejected_count, 4u);
}

// The case that motivated core PR #31: we opened a session, core expired it, and the
// status must follow core rather than our own record.
TEST_F(StreamServerTest, StatusFollowsCoreWhenTheSessionExpires) {
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "joint_impedance";
  req->timeout_s = 0.1;
  sink_.status.open = true;
  ASSERT_NE(call<Srv>("open_stream", req), nullptr);
  { const auto got = collect_status(300ms);
    ASSERT_FALSE(got.empty());
    EXPECT_TRUE(got.back().open);
    EXPECT_EQ(got.back().controller, "joint_impedance"); }

  sink_.status.open = false;          // core tore it down; nobody told us
  const auto got = collect_status(400ms);
  ASSERT_FALSE(got.empty());
  EXPECT_FALSE(got.back().open);
  EXPECT_EQ(got.back().controller, "");   // our label is dropped with it
}

TEST_F(StreamServerTest, StatusIsNotRepublishedWhileUnchanged) {
  const auto got = collect_status(700ms);
  EXPECT_LE(got.size(), 1u) << "status republished " << got.size()
                            << " times with nothing changing";
}
```

- [ ] **Step 2: Run to verify they fail**

Expected: FAIL — nothing is published, because `publish_status_if_changed` is a stub.

- [ ] **Step 3: Implement it**

Add a payload comparator to the anonymous namespace:
```cpp
bool same(const kinova_gen3_interfaces::msg::StreamStatus& a,
          const kinova_gen3_interfaces::msg::StreamStatus& b) {
  return a.open == b.open && a.controller == b.controller && a.channels == b.channels &&
         a.timeout_s == b.timeout_s && a.rejected_count == b.rejected_count;
}
```
and replace the stub:
```cpp
void StreamServer::publish_status_if_changed() {
  // open/timeout/rejected_count come from CORE, not from what we remember opening.
  // Without on_query_stream (core PR #31) this could only report the session we think
  // we have, which goes stale the moment the sampler expires one.
  const StreamStatus s = sink_.on_query_stream();
  StreamStatusMsg m;
  m.header.stamp = node_->now();
  m.open = s.open;
  m.timeout_s = s.timeout_s;
  m.rejected_count = s.rejected_count;
  { std::lock_guard<std::mutex> l(m_);
    // Core owns whether a session is open; `controller` is only our label for its
    // (kind, mode), so it is dropped as soon as core says closed.
    if (!s.open) open_controller_.clear();
    m.controller = open_controller_;
    for (const auto& r : registry())
      if (r.name == open_controller_) { m.channels = r.channels; break; }
    if (last_published_ && same(*last_published_, m)) return;
    last_published_ = m; }
  status_pub_->publish(m);
}
```

- [ ] **Step 4: Run the tests**

Expected: 12 tests, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/src/stream_server.cpp kinova_gen3_ros2/test/stream_server_test.cpp
git commit -m "feat(ros2): /stream_status reports core's session, not our record of it"
```

---

### Task 6: Wire `StreamServer` into `bringup_node`

**Files:**
- Modify: `kinova_gen3_ros2/src/bringup_node.cpp`

**Interfaces:**
- Consumes: `StreamServer` (Task 2), the existing `interface::Arbiter arb`.
- Produces: the running node exposes the streaming surface.

- [ ] **Step 1: Add the include**

After `#include "kinova_gen3_ros2/arbitration_server.h"`:
```cpp
#include "kinova_gen3_ros2/stream_server.h"
```

- [ ] **Step 2: Construct it beside `ArbitrationServer`**

After the `ArbitrationServer arbitration_server(...)` line:
```cpp
  // Wired to the Arbiter, not the Supervisor: that is how setpoint tokens get checked
  // at all. Declared here so it is destroyed before arb stops delegating.
  kinova_gen3_ros2::StreamServer stream_server(node, arb);
```

- [ ] **Step 3: Extend the start-up log**

Replace the `RCLCPP_INFO` "kinova_gen3_node up" call:
```cpp
  RCLCPP_INFO(node->get_logger(),
              "kinova_gen3_node up (%s); arbitration=%s; actions: /execute_joint_trajectory, "
              "/go_to_ee_pose, /go_to_joint_config, /go_to_preset; control: "
              "/acquire_control, /release_control, /revoke_control, /estop, /control_status; "
              "streaming: /open_stream, /close_stream, /list_controllers, /setpoint/*, "
              "/stream_status",
              use_sim ? "sim" : "real", mode_str.c_str());
```

- [ ] **Step 4: Build and run the existing end-to-end check**

Run: `./scripts/abra_colcon.sh --packages-up-to kinova_gen3_ros2`
Then: `ssh abra "bash /tmp/kinova-ros2-ws/src/kinova_gen3_ros2/scripts/abra_e2e_sim.sh 2>&1 | tail -5"`
Expected: `success_case=0 divergence_case=0` — the trajectory path is untouched.

- [ ] **Step 5: Verify the live surface**

```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver && \
  (ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf models/gen3_7dof_2f85.urdf &) && \
  sleep 6 && ros2 service call /list_controllers kinova_gen3_interfaces/srv/ListControllers {} | head -40 && \
  ros2 topic list | grep setpoint && \
  pkill -TERM -f kinova_gen3_node'"
```
Expected: seven controllers with `available` true for the first four, and six
`/setpoint/*` topics listed.

- [ ] **Step 6: Commit**

```bash
git add kinova_gen3_ros2/src/bringup_node.cpp
git commit -m "feat(ros2): wire StreamServer into the node"
```

---

### Task 7: End-to-end streaming test against a real Arbiter

**Files:**
- Modify: `kinova_gen3_ros2/test/arbitration_integration_test.cpp`

**Interfaces:**
- Consumes: core's `Arbiter`, the existing `RecordingSink`.
- Produces: nothing new.

- [ ] **Step 1: Extend `RecordingSink` to record setpoint tokens**

Add to the struct, beside the existing members:
```cpp
  Token last_setpoint_token{};
```
and replace `on_setpoint_joint_position`:
```cpp
  void on_setpoint_joint_position(const JointSetpoint& s) override {
    note("setpoint"); std::lock_guard<std::mutex> l(m); last_setpoint_token = s.token;
  }
```

- [ ] **Step 2: Write the failing tests**

```cpp
// Setpoints are gated exactly like goals -- and the rejection is SILENT, because
// on_setpoint_* returns void. rejected_count is the only signal there is.
TEST(ArbitrationIntegration, SetpointWithAStaleTokenIsDroppedSilently) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult first = arb.grant("teleop");
  ASSERT_TRUE(first.accepted);
  const GrantResult second = arb.grant("orchestrator");   // seizes; first token is dead
  ASSERT_TRUE(second.accepted);

  JointSetpoint stale; stale.token = first.token;
  arb.on_setpoint_joint_position(stale);
  EXPECT_FALSE(logged(sink, "setpoint"));                 // never reached the Supervisor
  const uint64_t rejected = arb.status().rejected_count;
  EXPECT_GT(rejected, 0u);

  JointSetpoint fresh; fresh.token = second.token;
  arb.on_setpoint_joint_position(fresh);
  EXPECT_TRUE(logged(sink, "setpoint"));
  EXPECT_EQ(sink.last_setpoint_token, second.token);
  EXPECT_EQ(arb.status().rejected_count, rejected);        // an accepted one adds nothing
}

// The scenario core's lock-free estop() was written for, and which the arbitration tier
// could only simulate: a real on_stream_open holding the arbiter mutex through its mode
// settle while an e-stop arrives.
TEST(ArbitrationIntegration, EstopHaltReachesTheArmDuringAStreamOpen) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  sink.block_open = true;
  StreamOpenRequest r;
  r.kind = SetpointKind::kJointPosition;
  r.control_mode = ControlModeKind::kPosition;
  r.timeout_s = 0.1;
  r.token = g.token;
  std::thread busy([&] { arb.on_stream_open(r); });   // blocks HOLDING the mutex
  std::this_thread::sleep_for(100ms);

  std::thread stopper([&] { arb.estop(); });
  bool halted = false;
  const auto t0 = std::chrono::steady_clock::now();
  double waited = 0.0;
  while (waited < 1.0) {
    if (logged(sink, "halt")) { halted = true; break; }
    std::this_thread::sleep_for(5ms);
    waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  EXPECT_TRUE(halted) << "the halt never reached the arm during a stream open";
  EXPECT_LT(waited, 0.5) << "halt queued behind the mode settle (" << waited << "s)";

  sink.block_open = false;
  sink.release_open = true;
  busy.join();
  stopper.join();
  EXPECT_TRUE(arb.status().estopped);
}
```
This needs two more members on `RecordingSink` and a blocking `on_stream_open`:
```cpp
  std::atomic<bool> block_open{false};
  std::atomic<bool> release_open{false};
```
```cpp
  StreamOpenResult on_stream_open(const StreamOpenRequest&) override {
    note("stream_open");
    while (block_open.load() && !release_open.load()) std::this_thread::sleep_for(1ms);
    return {};
  }
```

- [ ] **Step 3: Run to verify they fail, then pass**

Run: build, then `ssh abra "/tmp/rtest.sh arbitration_integration_test"`
Expected: the two new tests fail to compile first (`block_open` undeclared) until Step 2's
members are added; then 7 tests, 0 failures.

- [ ] **Step 4: Run the whole suite**

Run: `ssh abra "/tmp/rtest.sh"`
Expected: every test green, 65 existing plus the new ones.

- [ ] **Step 5: Commit**

```bash
git add kinova_gen3_ros2/test/arbitration_integration_test.cpp
git commit -m "test(ros2): setpoint tokens are gated, and e-stop beats the mode settle"
```

---

### Task 8: Documentation

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: nothing.

- [ ] **Step 1: Document the streaming surface**

In `README.md`, after the "Control-ownership services" section, add a "Streaming" section
covering: the `/open_stream` / `/close_stream` / `/list_controllers` services; the six
`/setpoint/*` topics with their message types; `/stream_status`; the controller table with
its available column; and the lifecycle:

```bash
ros2 service call /acquire_control kinova_gen3_interfaces/srv/AcquireControl "{owner_id: 'teleop'}"
ros2 service call /list_controllers kinova_gen3_interfaces/srv/ListControllers {}
# create your publisher and let discovery settle BEFORE opening
ros2 service call /open_stream kinova_gen3_interfaces/srv/OpenStream \
  "{controller: 'joint_impedance', timeout_s: 0.1, token: [...]}"
# publish on the returned channel faster than timeout_s
ros2 service call /close_stream kinova_gen3_interfaces/srv/CloseStream "{token: [...]}"
```

State the four rules that bite: one session at a time; the controller is fixed for its
lifetime (close-then-reopen re-pays the 250 ms mode settle); streams and trajectory goals
refuse each other in both directions; and a setpoint on the wrong channel is dropped,
counted, and **does not refresh the deadline**.

Note that setpoint topics are **best-effort**, so CLI subscribers must match:
`ros2 topic echo --qos-reliability best_effort /setpoint/joint_position`.

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs(ros2): the streaming tier — controllers, channels, session lifecycle"
```

---

## Deliberate test-scope calls

- **Core-side streaming behaviour is not re-tested here.** Deadline expiry, the
  hold-at-measured-q teardown, mode settling and the goal/stream mutual exclusion are all
  covered by core's `supervisor_test`. Rebuilding a `Supervisor` rig in this repo would
  re-test somebody else's unit.
- **`/setpoint/wrench` has no positive test** beyond "it does not reach core", because
  there is nothing for it to reach. That test exists as a guard against someone later
  wiring wrench into the wrong `on_setpoint_*` method.
