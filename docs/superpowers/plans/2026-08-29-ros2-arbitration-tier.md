# ROS 2 Arbitration Tier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose core's arbitration / control-ownership tier through ROS 2 — ownership
services, a broadcast e-stop, tokens on every motion-commanding message, and the status +
diagnostics surfaces that make it observable.

**Architecture:** Insert core's `interface::Arbiter` (a `CommandSink` decorator) between
the four existing action servers and the `Supervisor`. A new `ControlPlane` class owns the
ROS surface for `ArbitrationSink` and holds nothing else from core, so it unit-tests
against a fake with no robot. Tokens ride on goals; cancel replays the goal's stored token
because the ROS action protocol cannot carry one.

**Tech Stack:** ROS 2 Humble, `rclcpp` / `rclcpp_action`, `rosidl`, `diagnostic_updater`
(REP 107), GoogleTest via `ament_cmake_gtest`, `kinova_lowlevel` (core, C++17).

**Spec:** `docs/superpowers/specs/2026-08-29-ros2-arbitration-tier-design.md`

## Global Constraints

- **Core ref:** builds against `kinova-gen3-driver` **PR #29 (`feat/streaming-tier`)**, not
  `main`. Container builds need `make build CORE_REF=feat/streaming-tier`.
- **`arbitration_mode` default is `disabled`.** Every existing test and script must keep
  passing untouched. It is declared **read-only** — core has no setter.
- **Token type:** `uint8[16]` in IDL maps to `std::array<uint8_t,16>`, which *is*
  `kinova::interface::Token`. Assign directly; never memcpy or convert.
- **The token rule:** a token rides on every message that can command motion, and on
  nothing that stops the arm or reads state. `/estop` and `/revoke_control` carry none.
- **`/estop` QoS is reliable + VOLATILE.** A `transient_local` *subscription* is
  incompatible with the volatile publishers `ros2 topic pub` and rqt produce.
- **`/control_status` QoS is reliable + `transient_local`, depth 1.**
- **E-stop staleness is asymmetric:** `engaged:true` is never age-checked; `engaged:false`
  is. Unstamped (all-zero stamp) clears are accepted with a WARN so CLI use still works.
- **Build/test command** (from repo root, runs on `abra`):
  `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then `colcon test`.
- Match existing style: dense explanatory comments stating *why*, terse code.

---

### Task 1: Interface definitions

**Files:**
- Create: `kinova_arm_interfaces/msg/EStop.msg`
- Create: `kinova_arm_interfaces/msg/ControlStatus.msg`
- Create: `kinova_arm_interfaces/srv/AcquireControl.srv`
- Create: `kinova_arm_interfaces/srv/ReleaseControl.srv`
- Create: `kinova_arm_interfaces/srv/RevokeControl.srv`
- Modify: `kinova_arm_interfaces/CMakeLists.txt`
- Modify: `kinova_arm_interfaces/action/ExecuteJointTrajectory.action`
- Modify: `kinova_arm_interfaces/action/GoToEEPose.action`
- Modify: `kinova_arm_interfaces/action/GoToJointConfig.action`
- Modify: `kinova_arm_interfaces/action/GoToPreset.action`

**Interfaces:**
- Consumes: nothing.
- Produces: `kinova_arm_interfaces::msg::EStop` (header `msg/e_stop.hpp`),
  `msg::ControlStatus` (`msg/control_status.hpp`), `srv::AcquireControl`,
  `srv::ReleaseControl`, `srv::RevokeControl`, and a `uint8[16] token` field on all four
  action goals.

- [ ] **Step 1: Write `msg/EStop.msg`**

```
# Broadcast emergency stop. ANY node may publish; engaging is fail-safe by design.
#
# Deliberately NOT std_msgs/Bool: std_msgs primitives "do not convey semantic meaning
# about their contents" and are "NOT intended for long-term usage" (std_msgs docs).
# The stamp is load-bearing -- see the staleness policy in ControlPlane::on_estop.
std_msgs/Header header   # stamp: an all-zero stamp means "unstamped" and is accepted
bool   engaged           # true = engage the stop, false = clear it
string source            # who published it (node name / operator id), for the log
string reason            # free text, for the log
```

- [ ] **Step 2: Write `msg/ControlStatus.msg`**

```
# Who may command the arm right now. Published ON CHANGE, latched (transient_local),
# so a client that starts late or reconnects learns the current state immediately.
# Maps 1:1 onto kinova::interface::ArbitrationStatus.
#
# `generation` is how a client detects it was dispossessed: engaging the e-stop and
# every acquire both clear ownership, silently invalidating outstanding tokens.
std_msgs/Header header
bool   arbitration_enabled   # false = tokens are ignored (launch-time parameter)
bool   estopped              # latched; nothing is admitted
bool   owned                 # does anyone hold the arm?
string owner_id              # "" if none
uint64 generation            # bumps on every grant
uint64 rejected_count        # commands refused since start
```

- [ ] **Step 3: Write the three service definitions**

`srv/AcquireControl.srv`:
```
# Acquire control of the arm. SEIZES: if another owner holds the arm this succeeds
# anyway, halting their in-flight motion (settled -9). By operational contract this
# is called by the task orchestrator and nothing else.
string owner_id      # human-readable label; mirrors the actions' sender_id
---
bool      accepted
uint8[16] token      # the capability; put it on every motion-commanding goal
uint64    generation
string    message
```

`srv/ReleaseControl.srv`:
```
# Release control you hold. Refused unless the token matches the current owner.
uint8[16] token
---
bool   released
string message
```

`srv/RevokeControl.srv`:
```
# Operator override: take the arm from whoever holds it. Deliberately carries NO
# token -- this is the recovery path for a crashed owner, since ownership has no lease.
string reason
---
bool   revoked
string message
```

- [ ] **Step 4: Add `uint8[16] token` to all four action goals**

In `ExecuteJointTrajectory.action`, after the `string sender_id` line:
```
string  sender_id
uint8[16] token             # arbitration capability from /acquire_control; zeros = none
```
and extend the result-code comment:
```
int32   error_code          # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4, GOAL_TOLERANCE_VIOLATED=-5, PREEMPTED=-6, NOT_AUTHORIZED=-8, HALTED=-9
```

In `GoToEEPose.action`, `GoToJointConfig.action` and `GoToPreset.action`, after each
`string sender_id` line add the identical field:
```
uint8[16] token                    # arbitration capability from /acquire_control
```
and append `, NOT_AUTHORIZED=-8, HALTED=-9` to each `error_code` comment.

- [ ] **Step 5: Register the new files in `kinova_arm_interfaces/CMakeLists.txt`**

Extend the `rosidl_generate_interfaces` call:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointTrajectory.action"
  "action/GoToEEPose.action"
  "action/GoToJointConfig.action"
  "action/GoToPreset.action"
  "msg/JointImpedanceGains.msg"
  "msg/ControlStatus.msg"
  "msg/EStop.msg"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/RevokeControl.srv"
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs control_msgs action_msgs geometry_msgs)
```

- [ ] **Step 6: Build and verify the generated types**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_interfaces`
Expected: build succeeds.

Then confirm the generated names (this also pins the `EStop` → `e_stop.hpp` mapping the
later tasks `#include`):
```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  ros2 interface show kinova_arm_interfaces/msg/EStop && \
  ros2 interface show kinova_arm_interfaces/srv/AcquireControl && \
  ls /tmp/kinova-ros2-ws/install/kinova_arm_interfaces/include/kinova_arm_interfaces/kinova_arm_interfaces/msg/ | grep -i stop'"
```
Expected: both definitions print, and the header is `e_stop.hpp`. **If the header is named
differently, use that name in Tasks 2–5.**

- [ ] **Step 7: Commit**

```bash
git add kinova_arm_interfaces/
git commit -m "feat(interfaces): EStop, ControlStatus, ownership services, tokens on goals"
```

---

### Task 2: `ControlPlane` — ownership services

**Files:**
- Create: `kinova_arm_ros2/include/kinova_arm_ros2/control_plane.h`
- Create: `kinova_arm_ros2/src/control_plane.cpp`
- Create: `kinova_arm_ros2/test/fake_arbitration_sink.h`
- Create: `kinova_arm_ros2/test/control_plane_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt`
- Modify: `kinova_arm_ros2/package.xml`

**Interfaces:**
- Consumes: Task 1's `srv::AcquireControl`, `srv::ReleaseControl`, `srv::RevokeControl`,
  `msg::ControlStatus`, `msg::EStop`.
- Produces: `kinova_arm_ros2::ControlPlane`, constructed as
  `ControlPlane(rclcpp::Node::SharedPtr, kinova::interface::ArbitrationSink&, const std::string& hardware_id, double estop_clear_max_age_s)`,
  with one public method `void publish_status_if_changed()`. Task 8 constructs it.
  Also produces `FakeArbitrationSink` (test-only) used by Tasks 3–5.

- [ ] **Step 1: Write the fake sink**

`kinova_arm_ros2/test/fake_arbitration_sink.h`:
```cpp
#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter. Records the calls ControlPlane makes and maintains just
// enough state for status() to be meaningful. No Supervisor, no modes, no robot --
// which is the whole reason ControlPlane holds only an ArbitrationSink&.
struct FakeArbitrationSink : public kinova::interface::ArbitrationSink {
  mutable std::mutex m;
  std::vector<std::string> calls;
  kinova::interface::ArbitrationStatus st{};
  kinova::interface::Token next_token{};
  bool grant_accepted = true;

  void note(const std::string& s) { std::lock_guard<std::mutex> l(m); calls.push_back(s); }
  std::vector<std::string> log() const { std::lock_guard<std::mutex> l(m); return calls; }

  kinova::interface::GrantResult grant(const std::string& owner_id) override {
    note("grant:" + owner_id);
    if (!grant_accepted) return {false, kinova::interface::Token{}, st.generation, "refused"};
    st.owned = true; st.owner_id = owner_id; ++st.generation;
    return {true, next_token, st.generation, ""};
  }
  void revoke() override { note("revoke"); st.owned = false; st.owner_id.clear(); }
  void estop() override {
    note("estop"); st.estopped = true; st.owned = false; st.owner_id.clear();
  }
  void estop_clear() override { note("estop_clear"); st.estopped = false; }
  kinova::interface::ArbitrationStatus status() const override {
    std::lock_guard<std::mutex> l(m); return st;
  }
};
```

- [ ] **Step 2: Write the failing test**

`kinova_arm_ros2/test/control_plane_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "kinova_arm_ros2/control_plane.h"
#include "fake_arbitration_sink.h"
using namespace std::chrono_literals;

namespace {
// Cancels and joins the spin thread on ANY exit, so an early return from a failed
// ASSERT_* cannot destroy a joinable thread (std::terminate replaces the gtest
// diagnostic with a bare SIGABRT). Mirrors goto_ee_pose_integration_test.
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

class ControlPlaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("control_plane_test");
    sink_.next_token = mktoken(0xAB);
    plane_ = std::make_unique<kinova_arm_ros2::ControlPlane>(node_, sink_, "test", 1.0);
    ex_.add_node(node_);
  }
  // Calls a service and returns the response, or nullptr on timeout.
  template <class SrvT>
  typename SrvT::Response::SharedPtr call(const std::string& name,
                                          typename SrvT::Request::SharedPtr req) {
    auto client = node_->create_client<SrvT>(name);
    SpinThread spin(ex_);
    if (!client->wait_for_service(2s)) return nullptr;
    auto fut = client->async_send_request(req);
    if (fut.wait_for(2s) != std::future_status::ready) return nullptr;
    return fut.get();
  }
  rclcpp::Node::SharedPtr node_;
  FakeArbitrationSink sink_;
  std::unique_ptr<kinova_arm_ros2::ControlPlane> plane_;
  rclcpp::executors::MultiThreadedExecutor ex_;
};
}  // namespace

TEST_F(ControlPlaneTest, AcquireGrantsAndReturnsTheToken) {
  using Srv = kinova_arm_interfaces::srv::AcquireControl;
  auto req = std::make_shared<Srv::Request>();
  req->owner_id = "orchestrator";
  auto resp = call<Srv>("acquire_control", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->accepted);
  EXPECT_EQ(resp->token, mktoken(0xAB));
  EXPECT_EQ(resp->generation, 1u);
  EXPECT_EQ(sink_.log(), std::vector<std::string>{"grant:orchestrator"});
}

TEST_F(ControlPlaneTest, ReleaseWithMatchingTokenRevokes) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);

  using Rel = kinova_arm_interfaces::srv::ReleaseControl;
  auto rreq = std::make_shared<Rel::Request>();
  rreq->token = mktoken(0xAB);
  auto resp = call<Rel>("release_control", rreq);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->released);
  EXPECT_EQ(sink_.log().back(), "revoke");
}

// ArbitrationStatus deliberately does not carry the token, so ControlPlane checks
// against the one it minted. A stranger's token must not release someone else's arm.
TEST_F(ControlPlaneTest, ReleaseWithWrongTokenIsRefusedAndDoesNotRevoke) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);

  using Rel = kinova_arm_interfaces::srv::ReleaseControl;
  auto rreq = std::make_shared<Rel::Request>();
  rreq->token = mktoken(0x99);          // not the minted one
  auto resp = call<Rel>("release_control", rreq);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->released);
  EXPECT_EQ(sink_.log(), std::vector<std::string>{"grant:orchestrator"});   // no revoke
}

TEST_F(ControlPlaneTest, RevokeNeedsNoTokenAndAlwaysRevokes) {
  using Srv = kinova_arm_interfaces::srv::RevokeControl;
  auto req = std::make_shared<Srv::Request>();
  req->reason = "client hung";
  auto resp = call<Srv>("revoke_control", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->revoked);
  EXPECT_EQ(sink_.log().back(), "revoke");
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2`
Expected: FAIL — `kinova_arm_ros2/control_plane.h: No such file or directory`.

- [ ] **Step 4: Write the header**

`kinova_arm_ros2/include/kinova_arm_ros2/control_plane.h`:
```cpp
// kinova_arm_ros2/include/kinova_arm_ros2/control_plane.h
#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_arm_interfaces/msg/control_status.hpp"
#include "kinova_arm_interfaces/msg/e_stop.hpp"
#include "kinova_arm_interfaces/srv/acquire_control.hpp"
#include "kinova_arm_interfaces/srv/release_control.hpp"
#include "kinova_arm_interfaces/srv/revoke_control.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

// The ROS face of core's ArbitrationSink: ownership services, the broadcast e-stop,
// /control_status and the REP 107 /diagnostics contribution.
//
// It holds an ArbitrationSink& and NOTHING else from core -- no Supervisor, no
// ControlMode, no Dynamics -- so it is unit-testable against a fake with no robot,
// no URDF and no threads. That is the same reason the Arbiter itself is a decorator
// rather than part of the Supervisor.
class ControlPlane {
 public:
  using AcquireControl = kinova_arm_interfaces::srv::AcquireControl;
  using ReleaseControl = kinova_arm_interfaces::srv::ReleaseControl;
  using RevokeControl  = kinova_arm_interfaces::srv::RevokeControl;
  using EStop          = kinova_arm_interfaces::msg::EStop;
  using ControlStatus  = kinova_arm_interfaces::msg::ControlStatus;

  ControlPlane(rclcpp::Node::SharedPtr node, kinova::interface::ArbitrationSink& arb,
               const std::string& hardware_id, double estop_clear_max_age_s);

  // Publish only if the arbiter's status differs from what was last sent. Called by
  // every mutating handler (so ownership changes are immediate) and by a 10 Hz timer
  // (so externally-caused changes such as rejected_count are not missed).
  void publish_status_if_changed();

 private:
  void on_acquire(const std::shared_ptr<AcquireControl::Request>,
                  std::shared_ptr<AcquireControl::Response>);
  void on_release(const std::shared_ptr<ReleaseControl::Request>,
                  std::shared_ptr<ReleaseControl::Response>);
  void on_revoke(const std::shared_ptr<RevokeControl::Request>,
                 std::shared_ptr<RevokeControl::Response>);
  void on_estop(const EStop::SharedPtr msg);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void forget_token();

  rclcpp::Node::SharedPtr node_;
  kinova::interface::ArbitrationSink& arb_;
  double estop_clear_max_age_s_;

  rclcpp::Service<AcquireControl>::SharedPtr acquire_srv_;
  rclcpp::Service<ReleaseControl>::SharedPtr release_srv_;
  rclcpp::Service<RevokeControl>::SharedPtr revoke_srv_;
  rclcpp::Subscription<EStop>::SharedPtr estop_sub_;
  rclcpp::Publisher<ControlStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr estop_group_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  std::mutex m_;
  kinova::interface::Token retained_token_{};   // the token we minted; see on_release
  bool have_retained_ = false;
  std::optional<ControlStatus> last_published_;
};
}  // namespace kinova_arm_ros2
```

- [ ] **Step 5: Write the implementation**

`kinova_arm_ros2/src/control_plane.cpp`:
```cpp
// kinova_arm_ros2/src/control_plane.cpp
#include "kinova_arm_ros2/control_plane.h"
#include <chrono>
#include <functional>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_arm_ros2 {
using namespace kinova::interface;
using std::placeholders::_1; using std::placeholders::_2;

namespace {
// Compare the PAYLOAD only. The header stamp changes every poll, so including it
// would make "on change" mean "at 10 Hz forever".
bool same(const kinova_arm_interfaces::msg::ControlStatus& a,
          const kinova_arm_interfaces::msg::ControlStatus& b) {
  return a.arbitration_enabled == b.arbitration_enabled && a.estopped == b.estopped &&
         a.owned == b.owned && a.owner_id == b.owner_id &&
         a.generation == b.generation && a.rejected_count == b.rejected_count;
}
}  // namespace

ControlPlane::ControlPlane(rclcpp::Node::SharedPtr node, ArbitrationSink& arb,
                           const std::string& hardware_id, double estop_clear_max_age_s)
    : node_(node), arb_(arb), estop_clear_max_age_s_(estop_clear_max_age_s) {
  acquire_srv_ = node_->create_service<AcquireControl>(
      "acquire_control", std::bind(&ControlPlane::on_acquire, this, _1, _2));
  release_srv_ = node_->create_service<ReleaseControl>(
      "release_control", std::bind(&ControlPlane::on_release, this, _1, _2));
  revoke_srv_ = node_->create_service<RevokeControl>(
      "revoke_control", std::bind(&ControlPlane::on_revoke, this, _1, _2));

  // LATCHED: a client that starts late or reconnects must learn owner/generation/
  // estopped immediately rather than waiting for the next change. Safe because we are
  // the publisher -- an offered transient_local is compatible with volatile subscribers.
  status_pub_ = node_->create_publisher<ControlStatus>(
      "control_status", rclcpp::QoS(1).reliable().transient_local());

  // /estop gets its OWN mutually-exclusive callback group. Arbiter::estop() is
  // deliberately lock-free so it cannot queue behind a delegated call holding the
  // arbiter mutex (today a cuRobo round-trip; after the streaming tier, a 250 ms mode
  // settle). Sharing a group with those would rebuild that stall in ROS.
  estop_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = estop_group_;
  // VOLATILE, deliberately. A transient_local SUBSCRIPTION is incompatible with a
  // volatile publisher -- which is what `ros2 topic pub` and rqt produce -- so
  // requesting durability here would make an operator's e-stop silently fail to
  // connect. Leading '/' keeps the topic global rather than node-namespaced.
  estop_sub_ = node_->create_subscription<EStop>(
      "/estop", rclcpp::QoS(10).reliable(), std::bind(&ControlPlane::on_estop, this, _1),
      opts);

  status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                           [this] { publish_status_if_changed(); });

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID(hardware_id);
  updater_->add("Arbitration", this, &ControlPlane::diagnostics);

  publish_status_if_changed();   // seed the latched topic
}

void ControlPlane::forget_token() {
  std::lock_guard<std::mutex> l(m_);
  have_retained_ = false;
  retained_token_ = Token{};
}

void ControlPlane::on_acquire(const std::shared_ptr<AcquireControl::Request> req,
                              std::shared_ptr<AcquireControl::Response> resp) {
  // Read BEFORE granting: grant() SEIZES. If someone already holds the arm they are
  // about to be dispossessed and their in-flight goal settled -9. In a high-trust
  // system an unexpected seizure is exactly the mistake this tier exists to catch,
  // so it is loud rather than inferable from a generation counter nobody watched.
  const ArbitrationStatus before = arb_.status();
  const GrantResult r = arb_.grant(req->owner_id);
  if (r.accepted && before.owned && before.owner_id != req->owner_id) {
    RCLCPP_WARN(node_->get_logger(),
                "control SEIZED: '%s' took the arm from '%s' (generation %llu -> %llu); "
                "any motion the previous owner had running is being halted",
                req->owner_id.c_str(), before.owner_id.c_str(),
                static_cast<unsigned long long>(before.generation),
                static_cast<unsigned long long>(r.generation));
  }
  resp->accepted = r.accepted;
  resp->token = r.token;          // Token IS std::array<uint8_t,16>; direct assign
  resp->generation = r.generation;
  resp->message = r.message;
  { std::lock_guard<std::mutex> l(m_);
    have_retained_ = r.accepted;
    retained_token_ = r.accepted ? r.token : Token{}; }
  publish_status_if_changed();
}

void ControlPlane::on_release(const std::shared_ptr<ReleaseControl::Request> req,
                              std::shared_ptr<ReleaseControl::Response> resp) {
  // ArbitrationStatus deliberately does NOT carry the token -- publishing a capability
  // on a status topic would defeat it -- so the check is against the token this class
  // minted. Sound because ControlPlane is the only caller of grant/revoke/estop.
  bool ok = false;
  { std::lock_guard<std::mutex> l(m_);
    ok = have_retained_ && req->token == retained_token_; }
  if (!ok) {
    resp->released = false;
    resp->message = "token does not match the current owner";
    RCLCPP_WARN(node_->get_logger(), "release_control refused: token mismatch");
    return;
  }
  arb_.revoke();
  forget_token();
  resp->released = true;
  resp->message = "";
  publish_status_if_changed();
}

void ControlPlane::on_revoke(const std::shared_ptr<RevokeControl::Request> req,
                             std::shared_ptr<RevokeControl::Response> resp) {
  // No token: this is the recovery path for a crashed owner, and ownership has no lease.
  RCLCPP_WARN(node_->get_logger(), "operator revoke: %s",
              req->reason.empty() ? "(no reason given)" : req->reason.c_str());
  arb_.revoke();
  forget_token();
  resp->revoked = true;
  resp->message = "";
  publish_status_if_changed();
}

void ControlPlane::on_estop(const EStop::SharedPtr msg) {
  if (msg->engaged) {
    // NEVER age-checked. Refusing an old stop because its clock looked wrong is
    // precisely the failure we must not build: a stale stop is still honoured.
    RCLCPP_WARN(node_->get_logger(), "E-STOP engaged by '%s': %s",
                msg->source.c_str(),
                msg->reason.empty() ? "(no reason given)" : msg->reason.c_str());
    arb_.estop();
    forget_token();          // estop() clears ownership inside the Arbiter too
    publish_status_if_changed();
    return;
  }

  // Clearing IS age-checked: it re-enables a stopped arm, so a `ros2 bag` replay or a
  // message delayed behind a network hiccup must not do it silently.
  const bool unstamped = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
  if (unstamped) {
    // Accepted anyway: `ros2 topic pub` leaves the stamp at zero, and an e-stop
    // control that cannot be driven from the CLI is worse than one that can be replayed.
    RCLCPP_WARN(node_->get_logger(),
                "unstamped /estop clear from '%s' accepted; stamp it to enable the "
                "staleness guard", msg->source.c_str());
  } else if (estop_clear_max_age_s_ > 0.0) {
    // Construct with the node's clock type: subtracting rclcpp::Time values of
    // different clock types throws, and the default for a raw stamp is not
    // guaranteed to match node_->now() under use_sim_time.
    const rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
    const double age = (node_->now() - stamp).seconds();
    if (age > estop_clear_max_age_s_) {
      RCLCPP_WARN(node_->get_logger(),
                  "IGNORING stale /estop clear from '%s' (age %.2fs > %.2fs); the arm "
                  "stays stopped", msg->source.c_str(), age, estop_clear_max_age_s_);
      return;
    }
  }
  RCLCPP_WARN(node_->get_logger(), "e-stop CLEARED by '%s'; the arm has no owner until "
              "someone re-acquires control", msg->source.c_str());
  arb_.estop_clear();
  publish_status_if_changed();
}

void ControlPlane::publish_status_if_changed() {
  const ArbitrationStatus s = arb_.status();
  ControlStatus m;
  m.header.stamp = node_->now();
  m.arbitration_enabled = (s.mode == ArbitrationMode::kEnforced);
  m.estopped = s.estopped;
  m.owned = s.owned;
  m.owner_id = s.owner_id;
  m.generation = s.generation;
  m.rejected_count = s.rejected_count;
  { std::lock_guard<std::mutex> l(m_);
    if (last_published_ && same(*last_published_, m)) return;
    last_published_ = m; }
  status_pub_->publish(m);
}

void ControlPlane::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const ArbitrationStatus s = arb_.status();
  const bool enforced = (s.mode == ArbitrationMode::kEnforced);
  if (s.estopped)                stat.summary(DiagnosticStatus::ERROR, "E-STOPPED");
  else if (enforced && !s.owned) stat.summary(DiagnosticStatus::WARN, "no owner");
  else                           stat.summary(DiagnosticStatus::OK, "OK");
  // REP 107 names KeyValues as the place for "error counts, and information on latest
  // errors or timeouts".
  stat.add("arbitration_mode", enforced ? "enforced" : "disabled");
  stat.add("estopped", s.estopped);
  stat.add("owned", s.owned);
  stat.add("owner_id", s.owner_id.empty() ? std::string("(none)") : s.owner_id);
  stat.add("generation", static_cast<int>(s.generation));
  stat.add("rejected_count", static_cast<int>(s.rejected_count));
}
}  // namespace kinova_arm_ros2
```

- [ ] **Step 6: Add the build targets and dependencies**

In `kinova_arm_ros2/package.xml`, after the `<depend>sensor_msgs</depend>` line:
```xml
  <depend>diagnostic_updater</depend>
  <depend>diagnostic_msgs</depend>
  <depend>std_msgs</depend>
```

In `kinova_arm_ros2/CMakeLists.txt`, after the `find_package(rammp_curobo_interfaces REQUIRED)` line:
```cmake
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
```
and after the `ros2_backend` library block:
```cmake
add_library(control_plane src/control_plane.cpp)
target_include_directories(control_plane PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(control_plane
  rclcpp kinova_arm_interfaces diagnostic_updater diagnostic_msgs std_msgs)
target_link_libraries(control_plane kinova_lowlevel::kinova_lowlevel)
```
and inside the `if(BUILD_TESTING)` block:
```cmake
  ament_add_gtest(control_plane_test test/control_plane_test.cpp)
  target_include_directories(control_plane_test PRIVATE test)
  target_link_libraries(control_plane_test control_plane)
  ament_target_dependencies(control_plane_test
    rclcpp kinova_arm_interfaces diagnostic_updater diagnostic_msgs)
```
and add `control_plane` to the `kinova_arm_node` `target_link_libraries` list.

- [ ] **Step 7: Run the tests to verify they pass**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then
```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && cd /tmp/kinova-ros2-ws && \
  colcon test --packages-select kinova_arm_ros2 --ctest-args -R control_plane_test && \
  colcon test-result --all --verbose'"
```
Expected: 4 tests, 0 failures.

- [ ] **Step 8: Commit**

```bash
git add kinova_arm_ros2/include/kinova_arm_ros2/control_plane.h \
        kinova_arm_ros2/src/control_plane.cpp \
        kinova_arm_ros2/test/control_plane_test.cpp \
        kinova_arm_ros2/test/fake_arbitration_sink.h \
        kinova_arm_ros2/CMakeLists.txt kinova_arm_ros2/package.xml
git commit -m "feat(ros2): ControlPlane — acquire/release/revoke over the ArbitrationSink"
```

---

### Task 3: `/estop` staleness policy

**Files:**
- Modify: `kinova_arm_ros2/test/control_plane_test.cpp`

**Interfaces:**
- Consumes: `ControlPlane` and `FakeArbitrationSink` from Task 2. The implementation
  already shipped in Task 2 Step 5; this task proves the asymmetry and pins it.
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Append to `control_plane_test.cpp`, and add a publisher helper to the fixture first — put
this method inside `ControlPlaneTest`:
```cpp
  // Publishes on /estop and spins until ControlPlane has had a chance to handle it.
  void publish_estop(bool engaged, const rclcpp::Time& stamp, const std::string& src) {
    auto pub = node_->create_publisher<kinova_arm_interfaces::msg::EStop>(
        "/estop", rclcpp::QoS(10).reliable());
    kinova_arm_interfaces::msg::EStop m;
    m.header.stamp = stamp;
    m.engaged = engaged;
    m.source = src;
    SpinThread spin(ex_);
    // Wait for the subscription to match before publishing, else the message is
    // dropped into a void and the test races.
    for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i)
      std::this_thread::sleep_for(10ms);
    pub->publish(m);
    std::this_thread::sleep_for(200ms);
  }
```
and the tests:
```cpp
TEST_F(ControlPlaneTest, EstopEngageCallsEstop) {
  publish_estop(true, node_->now(), "operator");
  EXPECT_EQ(sink_.log().back(), "estop");
}

TEST_F(ControlPlaneTest, FreshEstopClearCallsEstopClear) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, node_->now(), "operator");
  EXPECT_EQ(sink_.log().back(), "estop_clear");
}

// A bag replay, or a clear delayed behind a network hiccup, must not re-enable a
// stopped arm. estop_clear_max_age_s is 1.0 in this fixture.
TEST_F(ControlPlaneTest, StaleEstopClearIsIgnored) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, node_->now() - rclcpp::Duration::from_seconds(30.0), "replay");
  EXPECT_EQ(sink_.log().back(), "estop");        // still stopped; no estop_clear
}

// The asymmetry, asserted so nobody later "tidies" it into a symmetric check:
// a stale STOP is still honoured. Both branches must fail toward "arm stays stopped".
TEST_F(ControlPlaneTest, StaleEstopEngageIsStillHonoured) {
  publish_estop(true, node_->now() - rclcpp::Duration::from_seconds(30.0), "replay");
  EXPECT_EQ(sink_.log().back(), "estop");
}

// `ros2 topic pub` leaves the stamp at zero. An e-stop control that cannot be driven
// from the CLI is worse than one that can be replayed.
TEST_F(ControlPlaneTest, UnstampedEstopClearIsAccepted) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, rclcpp::Time(0, 0, RCL_ROS_TIME), "cli");
  EXPECT_EQ(sink_.log().back(), "estop_clear");
}

// Engaging the e-stop clears ownership inside the Arbiter, so the token we retained
// is dead and a later release must not be honoured against a new owner.
TEST_F(ControlPlaneTest, EstopForgetsTheRetainedToken) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);
  publish_estop(true, node_->now(), "operator");

  using Rel = kinova_arm_interfaces::srv::ReleaseControl;
  auto rreq = std::make_shared<Rel::Request>();
  rreq->token = mktoken(0xAB);
  auto resp = call<Rel>("release_control", rreq);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->released);
}
```

- [ ] **Step 2: Run the tests**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then the
`control_plane_test` command from Task 2 Step 7.
Expected: 10 tests, 0 failures. (If `StaleEstopClearIsIgnored` fails, the age branch in
`on_estop` is wrong — it must `return` *before* `arb_.estop_clear()`.)

- [ ] **Step 3: Commit**

```bash
git add kinova_arm_ros2/test/control_plane_test.cpp
git commit -m "test(ros2): /estop staleness is asymmetric — stale stops honoured, stale clears refused"
```

---

### Task 4: `/control_status` on-change publishing and the seizure warning

**Files:**
- Modify: `kinova_arm_ros2/test/control_plane_test.cpp`

**Interfaces:**
- Consumes: `ControlPlane` from Task 2 (implementation already present).
- Produces: nothing new.

- [ ] **Step 1: Write the failing tests**

Add this helper method to `ControlPlaneTest`:
```cpp
  // Collects /control_status messages. transient_local matches the publisher, so a
  // subscriber created after the fact still receives the latest state.
  std::vector<kinova_arm_interfaces::msg::ControlStatus> collect_status(
      std::chrono::milliseconds dwell) {
    std::vector<kinova_arm_interfaces::msg::ControlStatus> got;
    std::mutex gm;
    auto sub = node_->create_subscription<kinova_arm_interfaces::msg::ControlStatus>(
        "control_status", rclcpp::QoS(10).reliable().transient_local(),
        [&got, &gm](kinova_arm_interfaces::msg::ControlStatus::SharedPtr m) {
          std::lock_guard<std::mutex> l(gm); got.push_back(*m);
        });
    SpinThread spin(ex_);
    std::this_thread::sleep_for(dwell);
    std::lock_guard<std::mutex> l(gm);
    return got;
  }
```
and the tests:
```cpp
// The 10 Hz timer must NOT republish an unchanged status, or "on change" means
// "at 10 Hz forever" and the header stamp is the only thing that ever differs.
TEST_F(ControlPlaneTest, StatusIsNotRepublishedWhileUnchanged) {
  const auto got = collect_status(600ms);
  EXPECT_LE(got.size(), 1u) << "status republished " << got.size()
                            << " times with nothing changing";
}

TEST_F(ControlPlaneTest, StatusPublishesOnOwnershipChange) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);
  const auto got = collect_status(300ms);
  ASSERT_FALSE(got.empty());
  EXPECT_TRUE(got.back().owned);
  EXPECT_EQ(got.back().owner_id, "orchestrator");
  EXPECT_EQ(got.back().generation, 1u);
}

// A late subscriber must learn the current state without waiting for a change --
// this is what /control_status being latched buys, and what lets a reconnecting
// client discover it was dispossessed.
TEST_F(ControlPlaneTest, LateSubscriberReceivesLatchedStatus) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);
  std::this_thread::sleep_for(200ms);
  const auto got = collect_status(400ms);   // subscribes only now
  ASSERT_FALSE(got.empty());
  EXPECT_TRUE(got.back().owned);
}

// Acquiring while someone else holds the arm SEIZES it: grant() succeeds, the
// incumbent is dispossessed, and generation bumps. Asserted so the behaviour is a
// decision rather than a surprise.
TEST_F(ControlPlaneTest, AcquireSeizesFromAnIncumbent) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto first = std::make_shared<Acq::Request>();
  first->owner_id = "teleop";
  ASSERT_NE(call<Acq>("acquire_control", first), nullptr);

  auto second = std::make_shared<Acq::Request>();
  second->owner_id = "orchestrator";
  auto resp = call<Acq>("acquire_control", second);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->accepted);
  EXPECT_EQ(resp->generation, 2u);
  const auto got = collect_status(300ms);
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.back().owner_id, "orchestrator");
}
```

- [ ] **Step 2: Run the tests**

Run: build, then the `control_plane_test` command from Task 2 Step 7.
Expected: 14 tests, 0 failures.

- [ ] **Step 3: Commit**

```bash
git add kinova_arm_ros2/test/control_plane_test.cpp
git commit -m "test(ros2): /control_status publishes on change, latches, and records seizure"
```

---

### Task 5: REP 107 `/diagnostics`

**Files:**
- Modify: `kinova_arm_ros2/test/control_plane_test.cpp`

**Interfaces:**
- Consumes: `ControlPlane` from Task 2 (implementation already present).
- Produces: nothing new.

- [ ] **Step 1: Write the failing test**

```cpp
// REP 107: reporting is on /diagnostics using diagnostic_msgs/DiagnosticArray at 1 Hz.
// Level must be ERROR while e-stopped -- this is what a monitoring dashboard reads.
TEST_F(ControlPlaneTest, DiagnosticsReportsErrorWhileEstopped) {
  publish_estop(true, node_->now(), "operator");

  std::vector<diagnostic_msgs::msg::DiagnosticArray> got;
  std::mutex gm;
  auto sub = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [&got, &gm](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {
        std::lock_guard<std::mutex> l(gm); got.push_back(*m);
      });
  { SpinThread spin(ex_); std::this_thread::sleep_for(2500ms); }   // >= 2 updater ticks

  std::lock_guard<std::mutex> l(gm);
  ASSERT_FALSE(got.empty()) << "nothing published on /diagnostics";
  bool found = false;
  for (const auto& arr : got)
    for (const auto& st : arr.status)
      if (st.name.find("Arbitration") != std::string::npos) {
        found = true;
        EXPECT_EQ(st.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
        EXPECT_EQ(st.hardware_id, "test");
        bool has_rejected = false;
        for (const auto& kv : st.values) if (kv.key == "rejected_count") has_rejected = true;
        EXPECT_TRUE(has_rejected) << "REP 107 wants error counts as KeyValues";
      }
  EXPECT_TRUE(found) << "no DiagnosticStatus named '...Arbitration'";
}
```
Add `#include "diagnostic_msgs/msg/diagnostic_array.hpp"` to the test's includes.

- [ ] **Step 2: Run the test**

Run: build, then the `control_plane_test` command from Task 2 Step 7.
Expected: 15 tests, 0 failures.

> **If it fails on the name:** the test matches `Arbitration` as a substring precisely
> because `diagnostic_updater` may or may not prefix the node name. If nothing is found at
> all, check that the `Updater` was constructed with the node (it owns its own 1 Hz timer)
> and that the executor is spinning for at least two ticks.

- [ ] **Step 3: Commit**

```bash
git add kinova_arm_ros2/test/control_plane_test.cpp
git commit -m "test(ros2): REP 107 diagnostics — ERROR while e-stopped, counts as KeyValues"
```

---

### Task 6: Token through the `ExecuteJointTrajectory` path

**Files:**
- Modify: `kinova_arm_ros2/src/message_mapping.cpp` (in `to_trajectory_goal`)
- Modify: `kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h:38` (`handles_`)
- Modify: `kinova_arm_ros2/src/ros2_backend.cpp` (`handle_cancel`, `handle_accepted`)
- Modify: `kinova_arm_ros2/test/message_mapping_test.cpp`

**Interfaces:**
- Consumes: Task 1's `token` field on `ExecuteJointTrajectory::Goal`.
- Produces: `Ros2Backend` now stores a `kinova::interface::Token` per accepted goal and
  replays it on cancel. `to_trajectory_goal(const ExecuteJointTrajectory::Goal&)` copies
  `g.token` into `tg.token`.

- [ ] **Step 1: Write the failing test**

Append to `kinova_arm_ros2/test/message_mapping_test.cpp`:
```cpp
// The file already has `using namespace kinova_arm_ros2;` at the top, so
// to_trajectory_goal is called unqualified, and goals are spelled out in full.
TEST(MessageMapping, CarriesTheArbitrationToken) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.assign(7, 0.0);
  g.trajectory.points.push_back(p);
  g.token.fill(0);
  g.token[0] = 0xAB;
  g.token[15] = 0xCD;
  const auto tg = kinova_arm_ros2::to_trajectory_goal(g);
  EXPECT_EQ(tg.token[0], 0xAB);
  EXPECT_EQ(tg.token[15], 0xCD);
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2`
Expected: FAIL — `tg.token` is all zeros (the mapping drops it).

- [ ] **Step 3: Copy the token in the mapping**

In `kinova_arm_ros2/src/message_mapping.cpp`, in `to_trajectory_goal`, after
`tg.sender_id = g.sender_id;`:
```cpp
  // uint8[16] generates as std::array<uint8_t,16>, which IS interface::Token.
  tg.token = g.token;
```

- [ ] **Step 4: Store the token per goal and replay it on cancel**

In `kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h`, replace the `handles_`
member with an entry that carries the token:
```cpp
  // The goal's token, kept so cancel can replay it. A ROS action cancel carries no
  // payload (action_msgs/CancelGoal is one GoalInfo), so without this a cancel would
  // arrive at the Arbiter with a zero token and be REFUSED under kEnforced -- the
  // motion would keep running while the client believed it had cancelled.
  struct Entry { std::shared_ptr<GoalHandle> gh; kinova::interface::Token token{}; };
  std::map<kinova::interface::GoalId, Entry> handles_;   // GoalId == GoalUUID
```

In `kinova_arm_ros2/src/ros2_backend.cpp`, `handle_accepted`:
```cpp
void Ros2Backend::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  const TrajectoryGoal tg = to_trajectory_goal(*gh->get_goal());
  { std::lock_guard<std::mutex> l(m_); handles_[id] = Entry{gh, tg.token}; }
  sink_->on_trajectory_accepted(id, tg);
}
```
`handle_cancel`:
```cpp
rclcpp_action::CancelResponse Ros2Backend::handle_cancel(std::shared_ptr<GoalHandle> gh) {
  if (!sink_) return rclcpp_action::CancelResponse::REJECT;
  const GoalId id = gh->get_goal_id();
  // Replay the token the goal was accepted with. This is FUNCTIONAL, not a security
  // measure: a ROS cancel is unauthenticated by protocol (see the spec, "Deliberate
  // deviations") -- but without a valid token the Arbiter refuses it outright.
  kinova::interface::Token token{};
  { std::lock_guard<std::mutex> l(m_);
    auto it = handles_.find(id);
    if (it != handles_.end()) token = it->second.token; }
  const CancelResponse r = sink_->on_trajectory_cancel({id, token});
  return (r == CancelResponse::kAccept) ? rclcpp_action::CancelResponse::ACCEPT
                                        : rclcpp_action::CancelResponse::REJECT;
}
```

Then fix the two other `handles_` readers, which now go through `.gh`:
- in `publish_feedback`: `gh = it->second.gh;` (already reads `it->second`; change to `it->second.gh`)
- in `settle`: same change.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then
```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && cd /tmp/kinova-ros2-ws && \
  colcon test --packages-select kinova_arm_ros2 && colcon test-result --all --verbose'"
```
Expected: all tests pass, including the 14 pre-existing `message_mapping_test` cases.

- [ ] **Step 6: Commit**

```bash
git add kinova_arm_ros2/src/message_mapping.cpp kinova_arm_ros2/src/ros2_backend.cpp \
        kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h \
        kinova_arm_ros2/test/message_mapping_test.cpp
git commit -m "feat(ros2): carry the arbitration token on trajectory goals and replay it on cancel"
```

---

### Task 7: Token through the `PlannedMoveServer` path

**Files:**
- Modify: `kinova_arm_ros2/include/kinova_arm_ros2/planned_move_server.h`
  (`struct Goal` at :268, `goals_[id] = Goal{gh, false}` at :146, the two
  `on_trajectory_cancel` call sites, and the `tg.sender_id` line)

**Interfaces:**
- Consumes: Task 1's `token` field on the three GoTo action goals.
- Produces: `PlannedMoveServer<ActionT>::Goal` gains a `kinova::interface::Token token`
  field; both cancel paths replay it.

- [ ] **Step 1: Add the token to the stored goal**

Replace the `Goal` struct (currently at :268):
```cpp
  // token: the goal's arbitration capability, kept so cancel can replay it. A ROS
  // action cancel carries no payload, so a cancel with a zero token would be REFUSED
  // by the Arbiter under kEnforced and the motion would run on.
  struct Goal { std::shared_ptr<GoalHandle> gh; bool executing = false;
                bool cancel_requested = false; kinova::interface::Token token{}; };
```

At :146, capture the token when the goal is recorded:
```cpp
    { std::lock_guard<std::mutex> l(m_);
      goals_[id] = Goal{gh, false, false, gh->get_goal()->token}; }
```

- [ ] **Step 2: Put the token on the planned trajectory**

Next to the existing `tg.sender_id` line:
```cpp
    kinova::interface::TrajectoryGoal tg = to_trajectory_goal(outcome.trajectory);
    tg.path_tolerance = kinova::JointVec::Constant(kGotoPathTolRad);
    tg.sender_id = gh->get_goal()->sender_id;
    tg.token     = gh->get_goal()->token;   // the plan inherits the goal's authority
```

- [ ] **Step 3: Replay the token at both cancel sites**

In `handle_cancel` (the `if (executing)` branch), read the token under the same lock that
already reads `executing`:
```cpp
    bool executing = false;
    kinova::interface::Token token{};
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it != goals_.end()) { executing = it->second.executing;
                                token = it->second.token;
                                it->second.cancel_requested = true; } }
    if (executing) {
      // Replay the goal's own token: a ROS cancel carries no payload, and a zero
      // token is refused under kEnforced.
      if (sink_) sink_->on_trajectory_cancel({id, token});   // Supervisor -> kPreempted -> settle()
    } else {
```

In `start_execution`'s re-issue at the end, read the token alongside `cancel_requested`:
```cpp
    bool canceled = false;
    kinova::interface::Token cancel_token{};
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it != goals_.end()) { canceled = it->second.cancel_requested;
                                cancel_token = it->second.token; } }
    if (canceled) sink_->on_trajectory_cancel({id, cancel_token});
```

- [ ] **Step 4: Run the full suite**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then the full
`colcon test` command from Task 6 Step 5.
Expected: all pre-existing integration tests still pass (they run with zero tokens, which
is what a `FakeSupervisor` ignores and what `kDisabled` admits).

- [ ] **Step 5: Commit**

```bash
git add kinova_arm_ros2/include/kinova_arm_ros2/planned_move_server.h
git commit -m "feat(ros2): PlannedMoveServer carries the goal token and replays it on cancel"
```

---

### Task 8: Wire the Arbiter into `bringup_node`

**Files:**
- Modify: `kinova_arm_ros2/src/bringup_node.cpp`

**Interfaces:**
- Consumes: `ControlPlane` (Task 2), core's `interface::Arbiter`.
- Produces: the running node exposes `/acquire_control`, `/release_control`,
  `/revoke_control`, `/estop`, `/control_status`, `/diagnostics`, and gates all four
  actions through the Arbiter.

- [ ] **Step 1: Add the includes**

After `#include "kinova_lowlevel/interface/supervisor.h"`:
```cpp
#include "kinova_lowlevel/interface/arbiter.h"
```
and after `#include "kinova_arm_ros2/goto_preset_server.h"`:
```cpp
#include "kinova_arm_ros2/control_plane.h"
```

- [ ] **Step 2: Declare the parameters**

Immediately after `auto node = std::make_shared<rclcpp::Node>("kinova_arm_node");`:
```cpp
  // READ-ONLY on purpose: core's ArbitrationMode is a constructor argument with no
  // setter, so a dynamic parameter would appear to work and silently do nothing.
  rcl_interfaces::msg::ParameterDescriptor mode_desc;
  mode_desc.read_only = true;
  mode_desc.description = "enforced | disabled. Launch-time only; core has no setter.";
  const std::string mode_str =
      node->declare_parameter("arbitration_mode", "disabled", mode_desc);
  if (mode_str != "enforced" && mode_str != "disabled") {
    RCLCPP_FATAL(node->get_logger(),
                 "arbitration_mode must be 'enforced' or 'disabled', got '%s'",
                 mode_str.c_str());
    return 1;
  }
  const auto arb_mode = (mode_str == "enforced") ? interface::ArbitrationMode::kEnforced
                                                 : interface::ArbitrationMode::kDisabled;
  const double estop_clear_max_age_s =
      node->declare_parameter("estop_clear_max_age_s", 1.0);
```
Add `#include "rcl_interfaces/msg/parameter_descriptor.hpp"` to the includes.

- [ ] **Step 3: Insert the Arbiter and ControlPlane**

Replace the four `set_command_sink(&sup)` lines. Declaration order matters — destruction
is reverse, and `control_plane` must stop accepting ROS calls before `arb` stops
delegating, which must happen before `sup` goes away:
```cpp
  interface::Supervisor sup(pos, imp, tau, exec, snap, pump_dyn, *backend, router);
  // Supervisor implements BOTH CommandSink and StreamSink, so it is passed twice --
  // the idiom core's own tests use (Arbiter arb{sink, sink, mode, seed}).
  interface::Arbiter arb(sup, sup, arb_mode);
  // Declared after arb so it is destroyed FIRST: it must stop accepting ROS calls
  // before arb stops delegating, and arb before sup goes away.
  kinova_arm_ros2::ControlPlane control_plane(
      node, arb, use_sim ? std::string("sim") : ip, estop_clear_max_age_s);
  // (`ip` and `use_sim` are the existing locals from the --ip / --sim arg parsing.)

  // Every command path now goes through the Arbiter rather than straight to the
  // Supervisor. Nothing else about the servers changes.
  backend->set_command_sink(&arb);
  goto_server.set_command_sink(&arb);
  jc_server.set_command_sink(&arb);
  preset_server.set_command_sink(&arb);
```

- [ ] **Step 4: Extend the start-up log**

Replace the existing `RCLCPP_INFO` "kinova_arm_node up" call:
```cpp
  RCLCPP_INFO(node->get_logger(),
              "kinova_arm_node up (%s); arbitration=%s; actions: /execute_joint_trajectory, "
              "/go_to_ee_pose, /go_to_joint_config, /go_to_preset; control: "
              "/acquire_control, /release_control, /revoke_control, /estop, /control_status",
              use_sim ? "sim" : "real", mode_str.c_str());
```

- [ ] **Step 5: Build and run the existing end-to-end check**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2`
Then: `ssh abra "bash /tmp/kinova-ros2-ws/src/kinova_arm_ros2/scripts/abra_e2e_sim.sh 2>&1 | tail -5"`
Expected: `success_case=0 divergence_case=0`. This is the backward-compatibility gate —
the default `disabled` mode must leave the existing path untouched.

- [ ] **Step 6: Verify the new surface exists**

```bash
ssh abra "bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/kinova-ros2-ws/install/setup.bash && \
  cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver && \
  (ros2 run kinova_arm_ros2 kinova_arm_node --sim --urdf models/gen3_7dof_2f85.urdf &) && \
  sleep 5 && ros2 service list | grep control && ros2 topic list | grep -E \"control_status|estop|diagnostics\" && \
  ros2 topic echo --once /control_status && \
  pkill -TERM -f kinova_arm_node'"
```
Expected: the three services and three topics are listed, and `/control_status` echoes
with `arbitration_enabled: false`.

- [ ] **Step 7: Commit**

```bash
git add kinova_arm_ros2/src/bringup_node.cpp
git commit -m "feat(ros2): wire the Arbiter and ControlPlane into the node"
```

---

### Task 9: End-to-end arbitration integration test

**Files:**
- Create: `kinova_arm_ros2/test/arbitration_integration_test.cpp`
- Modify: `kinova_arm_ros2/CMakeLists.txt`

**Interfaces:**
- Consumes: everything above, plus core's `interface::Arbiter`.
- Produces: nothing new.

This is the task that proves the two bugs the spec exists to prevent: a cancel silently
refused under `kEnforced`, and an e-stop that cannot get past a busy arbiter.

- [ ] **Step 1: Write the failing tests**

`kinova_arm_ros2/test/arbitration_integration_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "kinova_lowlevel/interface/arbiter.h"
#include "kinova_lowlevel/interface/ports.h"
using namespace kinova::interface;
using namespace std::chrono_literals;

namespace {
// Records what actually reached the Supervisor's side of the Arbiter, and can be told
// to block inside a delegated call so the e-stop path can be raced against it.
struct RecordingSink : public CommandSink, public StreamSink {
  std::mutex m;
  std::vector<std::string> calls;
  std::atomic<bool> block_goal{false};
  std::atomic<bool> release_goal{false};

  void note(const std::string& s) { std::lock_guard<std::mutex> l(m); calls.push_back(s); }
  std::vector<std::string> log() { std::lock_guard<std::mutex> l(m); return calls; }

  GoalResponse on_trajectory_goal(const TrajectoryGoal&) override {
    note("goal");
    while (block_goal.load() && !release_goal.load()) std::this_thread::sleep_for(1ms);
    return GoalResponse::kAccept;
  }
  void on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override { note("accepted"); }
  CancelResponse on_trajectory_cancel(const CancelRequest&) override {
    note("cancel"); return CancelResponse::kAccept;
  }
  GainsResult on_set_gains(const GainsRequest&) override { return {}; }
  ArmState on_query_state() override { ArmState s; s.stamp_s = 1.0; return s; }
  void on_halt(HaltReason) override { note("halt"); }

  StreamOpenResult on_stream_open(const StreamOpenRequest&) override { return {}; }
  void on_stream_close(const StreamCloseRequest&) override {}
  void on_setpoint_joint_position(const JointSetpoint&) override {}
  void on_setpoint_joint_velocity(const JointSetpoint&) override {}
  void on_setpoint_joint_torque(const JointSetpoint&) override {}
  void on_setpoint_pose(const PoseSetpoint&) override {}
  void on_setpoint_twist(const TwistSetpoint&) override {}
};
GoalId mkid(uint8_t x) { GoalId id{}; id[0] = x; return id; }
}  // namespace

// The regression gate for the stored-token replay. A cancel carrying the goal's token
// must be admitted; one carrying zeros must not.
TEST(ArbitrationIntegration, CancelWithTheGoalsTokenIsAdmittedUnderEnforced) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  EXPECT_EQ(arb.on_trajectory_cancel({mkid(1), g.token}), CancelResponse::kAccept);
  EXPECT_EQ(sink.log().back(), "cancel");
}

TEST(ArbitrationIntegration, CancelWithAZeroTokenIsRefusedUnderEnforced) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  ASSERT_TRUE(arb.grant("orchestrator").accepted);

  EXPECT_EQ(arb.on_trajectory_cancel({mkid(1), Token{}}), CancelResponse::kReject);
  for (const auto& c : sink.log()) EXPECT_NE(c, "cancel");   // never reached the Supervisor
}

// kDisabled admits the zero token: the backward-compatibility gate that keeps every
// existing script working under the default mode.
TEST(ArbitrationIntegration, ZeroTokenIsAdmittedUnderDisabled) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  TrajectoryGoal tg;
  EXPECT_EQ(arb.on_trajectory_goal(tg), GoalResponse::kAccept);
}

TEST(ArbitrationIntegration, StaleTokenIsRejectedAfterSeizure) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult first = arb.grant("teleop");
  const GrantResult second = arb.grant("orchestrator");
  ASSERT_TRUE(second.accepted);
  EXPECT_EQ(second.generation, first.generation + 1);

  TrajectoryGoal stale; stale.token = first.token;
  EXPECT_EQ(arb.on_trajectory_goal(stale), GoalResponse::kRejectUnauthorized);
  TrajectoryGoal fresh; fresh.token = second.token;
  EXPECT_EQ(arb.on_trajectory_goal(fresh), GoalResponse::kAccept);
}

// THE safety test.
//
// NOTE ON WHAT IS BEING MEASURED: estop() *does* block on the arbiter mutex at its
// tail, by design -- the ownership bookkeeping and the second halt delivery both run
// under m_. Timing how long `arb.estop()` takes to RETURN would therefore just measure
// the blocked delegate and always fail. The property core actually engineered is that
// the LATCH and the FIRST HALT both happen BEFORE estop() ever contends for the lock.
// So: run estop() on its own thread and watch for the halt arriving downstream while
// the delegate is still stuck holding the mutex.
TEST(ArbitrationIntegration, EstopHaltReachesTheArmWhileADelegatedCallBlocks) {
  RecordingSink sink;
  Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult g = arb.grant("orchestrator");
  ASSERT_TRUE(g.accepted);

  sink.block_goal = true;
  TrajectoryGoal tg; tg.token = g.token;
  std::thread busy([&] { arb.on_trajectory_goal(tg); });   // blocks HOLDING the mutex
  std::this_thread::sleep_for(100ms);

  std::thread stopper([&] { arb.estop(); });

  bool halted = false;
  const auto t0 = std::chrono::steady_clock::now();
  double waited = 0.0;
  while (waited < 1.0) {
    for (const auto& c : sink.log()) if (c == "halt") { halted = true; break; }
    if (halted) break;
    std::this_thread::sleep_for(5ms);
    waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }

  EXPECT_TRUE(halted) << "the halt never reached the arm while a delegate held the mutex";
  EXPECT_LT(waited, 0.5) << "halt queued behind the delegated call (" << waited << "s)";

  sink.block_goal = false;
  sink.release_goal = true;
  busy.join();
  stopper.join();
  EXPECT_TRUE(arb.status().estopped);
  // And nothing is admitted afterwards -- the latch is the one thing kDisabled
  // does not bypass either.
  EXPECT_EQ(arb.on_trajectory_goal(tg), GoalResponse::kRejectUnauthorized);
}
```

- [ ] **Step 2: Register the test**

In `kinova_arm_ros2/CMakeLists.txt`, inside `if(BUILD_TESTING)`:
```cmake
  ament_add_gtest(arbitration_integration_test test/arbitration_integration_test.cpp)
  target_link_libraries(arbitration_integration_test kinova_lowlevel::kinova_lowlevel)
```

- [ ] **Step 3: Run the tests**

Run: `./scripts/abra_colcon.sh --packages-select kinova_arm_ros2` then the full
`colcon test` command from Task 6 Step 5.
Expected: 5 new tests pass, and the whole suite is green.

- [ ] **Step 4: Full regression + end-to-end**

```bash
ssh abra "bash /tmp/kinova-ros2-ws/src/kinova_arm_ros2/scripts/abra_e2e_sim.sh 2>&1 | tail -5"
```
Expected: `success_case=0 divergence_case=0`, and `colcon test-result --all` reports zero
failures across every test in the package.

- [ ] **Step 5: Commit**

```bash
git add kinova_arm_ros2/test/arbitration_integration_test.cpp kinova_arm_ros2/CMakeLists.txt
git commit -m "test(ros2): arbitration end-to-end — token'd cancel, stale tokens, e-stop under load"
```

---

## Deliberate test-scope calls

Two items in the spec's testing section are **not** rebuilt here, on purpose:

- **"`/estop` settles an in-flight goal with `-9`."** That path is entirely inside core:
  `Supervisor`'s sampler settles the active and queued goals on halt, and core's
  `supervisor_test` already covers it. Reproducing it here would mean standing up modes,
  a URDF, an `RtExecutor` and threads to re-test somebody else's unit. Our half of the
  path — router to server to `terminal()` — is already covered by the existing
  `goto_*_integration_test` cases via `FakeSupervisor`.
- **"`/estop` has its own callback group."** Task 9 proves the property that matters (the
  halt is delivered before the arbiter mutex is contended) at the `Arbiter` level. That
  the ROS subscription is on a dedicated `MutuallyExclusive` group is verified by reading
  `control_plane.cpp`; asserting it from a test would mean reaching into rclcpp internals
  for little added confidence.

## Documentation (fold into Task 9's commit or its own)

- [ ] Update `docs/guide-goto-actions.md` and `README.md` with the new services/topics,
      the `arbitration_mode` parameter, and the acquire → command → release flow.
- [ ] Note in `README.md` that `/estop` is open to any publisher by design and that
      arbitration is cooperative coordination, not authorization (link the spec's
      "What arbitration is, and is not").
