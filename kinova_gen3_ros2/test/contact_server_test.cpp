// /set_contact_thresholds, /contact_monitor and the "Contact detection"
// diagnostics, against a fake contact sink behind core's real Arbiter
// (enforced), so the token gate is the real one. No robot.
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "kinova_gen3_ros2/contact_server.h"
#include "kinova_lowlevel/interface/arbiter.h"
#include "fake_contact_sink.h"
#include "fake_gripper_sink.h"
#include "fake_stream_sink.h"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using kinova_gen3_ros2::ContactServer;
using Srv = ContactServer::SetContactThresholds;
using Msg = ContactServer::ContactMonitorStateMsg;

namespace {
const double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Rig {
  FakeContactSink sink;
  FakeStreamSink stream;
  FakeGripperSink grip;
  kinova::interface::Arbiter arb{sink, stream, grip,
                                 kinova::interface::ArbitrationMode::kEnforced,
                                 /*seed=*/42};
  kinova::interface::Token token{};
  rclcpp::Node::SharedPtr node, client_node;
  std::unique_ptr<ContactServer> server;
  rclcpp::executors::MultiThreadedExecutor ex;
  std::thread spin;

  explicit Rig(const std::string &name, std::string detection = "enforce") {
    token = arb.grant("contact_server_test").token;
    node = std::make_shared<rclcpp::Node>(name);
    client_node = std::make_shared<rclcpp::Node>(name + "_client");
    server = std::make_unique<ContactServer>(node, arb, std::move(detection));
    ex.add_node(node);
    spin = std::thread([this] { ex.spin(); });
  }
  ~Rig() {
    ex.cancel();
    if (spin.joinable())
      spin.join();
  }
  std::shared_ptr<Srv::Response>
  call(uint8_t profile, std::vector<double> custom, bool with_token = true) {
    auto client = client_node->create_client<Srv>("set_contact_thresholds");
    if (!client->wait_for_service(5s)) {
      ADD_FAILURE() << "set_contact_thresholds never appeared";
      return nullptr;
    }
    auto req = std::make_shared<Srv::Request>();
    req->profile = profile;
    req->custom = std::move(custom);
    if (with_token)
      std::copy(token.begin(), token.end(), req->token.begin());
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(client_node, fut, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "no response within 5 s";
      return nullptr;
    }
    return fut.get();
  }
};

std::vector<double> to_std(const kinova::JointVec &v) {
  return std::vector<double>(v.data(), v.data() + v.size());
}
std::vector<double> to_std(const std::array<double, 7> &v) {
  return std::vector<double>(v.begin(), v.end());
}
bool has(const std::string &s, const std::string &part) {
  return s.find(part) != std::string::npos;
}
} // namespace

class ContactServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

// Review Focus 5: every bad request is refused with the problem named, and the
// active thresholds are unchanged.
TEST_F(ContactServerTest, RefusesBadRequests) {
  Rig r("contact_refuses");
  const auto normal = to_std(r.sink.profiles.normal);
  struct Case {
    const char *what;
    uint8_t profile;
    std::vector<double> custom;
    bool with_token;
    std::vector<std::string> expect; // substrings of the message
    bool reaches_core;
  };
  const std::vector<Case> cases = {
      {"custom with 6 values",
       Srv::Request::PROFILE_CUSTOM,
       {8, 8, 6, 6, 2, 2},
       true,
       {"exactly 7", "got 6"},
       false},
      {"custom with 8 values",
       Srv::Request::PROFILE_CUSTOM,
       {8, 8, 6, 6, 2, 2, 2, 2},
       true,
       {"exactly 7", "got 8"},
       false},
      {"values on a named profile",
       Srv::Request::PROFILE_LOOSE,
       {8, 8, 6, 6, 2, 2, 2},
       true,
       {"empty", "PROFILE_CUSTOM"},
       false},
      {"NaN",
       Srv::Request::PROFILE_CUSTOM,
       {8, 8, kNaN, 6, 2, 2, 2},
       true,
       {"joint 3", "finite"},
       false},
      {"infinity",
       Srv::Request::PROFILE_CUSTOM,
       {8, 8, 6, 6, 2, 2, INFINITY},
       true,
       {"joint 7", "finite"},
       false},
      {"unknown profile", 9, {}, true, {"unknown profile 9"}, false},
      {"above the ceiling",
       Srv::Request::PROFILE_CUSTOM,
       {8, 8, 6, 6, 7.5, 2, 2},
       true,
       {"joint 5", "ceiling 7"},
       true},
      {"zero",
       Srv::Request::PROFILE_CUSTOM,
       {0, 8, 6, 6, 2, 2, 2},
       true,
       {"joint 1", "> 0"},
       true},
      {"negative",
       Srv::Request::PROFILE_CUSTOM,
       {8, -1, 6, 6, 2, 2, 2},
       true,
       {"joint 2", "> 0"},
       true},
      {"zero token",
       Srv::Request::PROFILE_LOOSE,
       {},
       false,
       {"not authorized"},
       false},
  };
  for (const auto &c : cases) {
    const std::size_t before = r.sink.set_count();
    const auto res = r.call(c.profile, c.custom, c.with_token);
    ASSERT_TRUE(res) << c.what;
    EXPECT_FALSE(res->accepted) << c.what;
    for (const auto &e : c.expect)
      EXPECT_TRUE(has(res->message, e))
          << c.what << ": '" << res->message << "' lacks '" << e << "'";
    EXPECT_EQ(r.sink.set_count(), before + (c.reaches_core ? 1 : 0)) << c.what;
    // The response reports what is active, and that did not change.
    EXPECT_EQ(res->profile, Srv::Request::PROFILE_DEFAULT) << c.what;
    EXPECT_EQ(to_std(res->thresholds), normal) << c.what;
    EXPECT_EQ(to_std(r.sink.on_query_contact().status.theta), normal) << c.what;
  }
}

TEST_F(ContactServerTest, AppliesNamedProfile) {
  Rig r("contact_applies");
  auto res = r.call(Srv::Request::PROFILE_LOOSE, {});
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->accepted) << res->message;
  EXPECT_EQ(res->profile, Srv::Request::PROFILE_LOOSE);
  EXPECT_EQ(to_std(res->thresholds), to_std(r.sink.profiles.loose));
  ASSERT_EQ(r.sink.set_count(), 1u);
  EXPECT_EQ(r.sink.sets.back().profile,
            kinova::interface::ContactProfile::kLoose);

  const std::vector<double> custom = {10, 10, 8, 8, 3, 3, 3};
  res = r.call(Srv::Request::PROFILE_CUSTOM, custom);
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->accepted) << res->message;
  EXPECT_EQ(res->profile, Srv::Request::PROFILE_CUSTOM);
  EXPECT_EQ(to_std(res->thresholds), custom);
}

TEST_F(ContactServerTest, PublishesState) {
  Rig r("contact_publishes");
  {
    std::lock_guard<std::mutex> l(r.sink.m);
    auto &s = r.sink.state;
    s.profile = kinova::interface::ContactProfile::kLoose;
    s.status.phase = kinova::ContactPhase::kLatched;
    s.status.r << 0.1, 21, 0.3, 0.4, 0.5, 0.6, 0.7;
    s.status.theta = r.sink.profiles.loose;
    s.status.would_trips = 3;
    s.status.bad_samples = 2;
    s.status.has_trip = true;
    s.status.last.joint = 1; // 0-based in core
    s.status.last.r << 0, 21.5, 0, 0, 0, 0, 0;
    s.status.last.theta = r.sink.profiles.loose;
    s.status.last.q << 1, 2, 3, 4, 5, 6, 7;
    s.status.last.qd << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  }
  Msg got;
  bool seen = false;
  std::mutex m;
  auto sub = r.client_node->create_subscription<Msg>(
      "/contact_monitor", rclcpp::SensorDataQoS(), [&](Msg::SharedPtr msg) {
        std::lock_guard<std::mutex> l(m);
        got = *msg;
        seen = true;
      });
  // The server's own 50 Hz timer publishes; nothing here calls publish_state.
  for (int i = 0; i < 100 && !seen; ++i)
    rclcpp::spin_some(r.client_node), std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(seen);
  EXPECT_EQ(got.detection, Msg::DETECTION_ENFORCE);
  EXPECT_EQ(got.phase, Msg::PHASE_LATCHED);
  EXPECT_EQ(got.profile, Srv::Request::PROFILE_LOOSE);
  EXPECT_DOUBLE_EQ(got.residual[1], 21.0);
  EXPECT_EQ(to_std(got.thresholds), to_std(r.sink.profiles.loose));
  EXPECT_EQ(got.would_trips, 3u);
  EXPECT_EQ(got.bad_samples, 2u);
  EXPECT_TRUE(got.has_trip);
  EXPECT_EQ(got.trip_joint, 2); // 1-based on the wire
  EXPECT_DOUBLE_EQ(got.trip_residual[1], 21.5);
  EXPECT_DOUBLE_EQ(got.trip_thresholds[1], 20.0);
  EXPECT_DOUBLE_EQ(got.trip_q[6], 7.0);
  EXPECT_DOUBLE_EQ(got.trip_qd[0], 0.1);

  // With detection off there is no monitor: the sink says enabled = false.
  const auto off = kinova_gen3_ros2::to_contact_monitor_msg({});
  EXPECT_EQ(off.detection, Msg::DETECTION_OFF);
  EXPECT_FALSE(off.has_trip);
  EXPECT_EQ(off.trip_joint, 0);
  kinova::interface::ContactState obs;
  obs.enabled = true;
  obs.status.detection = kinova::ContactDetection::kObserve;
  EXPECT_EQ(kinova_gen3_ros2::to_contact_monitor_msg(obs).detection,
            Msg::DETECTION_OBSERVE);
}

TEST(ContactDiagnostics, DiagnosticsEachPhase) {
  using kinova::ContactPhase;
  kinova::interface::ContactState s;
  s.enabled = true;
  s.profile = kinova::interface::ContactProfile::kDefault;
  s.status.theta << 8, 8, 6, 6, 2, 2, 2;
  auto diag = [](const kinova::interface::ContactState &st,
                 const std::string &det) {
    diagnostic_updater::DiagnosticStatusWrapper w;
    kinova_gen3_ros2::contact_diagnostics(st, det, w);
    return DiagnosticStatus(w); // the wrapper is not copyable
  };

  s.status.phase = ContactPhase::kSettling;
  auto w = diag(s, "enforce");
  EXPECT_EQ(w.level, DiagnosticStatus::WARN);
  EXPECT_EQ(w.message, "settling");

  s.status.phase = ContactPhase::kArmed;
  w = diag(s, "enforce");
  EXPECT_EQ(w.level, DiagnosticStatus::OK);
  EXPECT_EQ(w.message, "armed (profile default)");

  s.status.phase = ContactPhase::kLatched;
  s.status.has_trip = true;
  s.status.last.joint = 2;
  s.status.last.r << 0, 0, -6.4, 0, 0, 0, 0;
  s.status.last.theta = s.status.theta;
  w = diag(s, "enforce");
  EXPECT_EQ(w.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(w.message, "stopped on contact: joint 3, r = -6.40 N·m > θ = 6.00 "
                       "N·m (profile default); call /clear_faults");

  kinova::interface::ContactState obs = s;
  obs.status.detection = kinova::ContactDetection::kObserve;
  obs.status.phase = ContactPhase::kArmed;
  w = diag(obs, "observe");
  EXPECT_EQ(w.level, DiagnosticStatus::WARN);
  EXPECT_EQ(w.message, "observe mode: not stopping the arm");

  w = diag({}, "off");
  EXPECT_EQ(w.level, DiagnosticStatus::WARN);
  EXPECT_EQ(w.message, "DISABLED (contact_detection=off)");

  bool saw_phase = false, saw_trips = false;
  s.status.would_trips = 4;
  w = diag(s, "enforce");
  for (const auto &kv : w.values) {
    saw_phase |= kv.key == "phase" && kv.value == "latched";
    saw_trips |= kv.key == "would_trips" && kv.value == "4";
  }
  EXPECT_TRUE(saw_phase);
  EXPECT_TRUE(saw_trips);
}

// The diagnostics task is on /diagnostics under its own name.
TEST_F(ContactServerTest, DiagnosticsTaskIsPublished) {
  Rig r("contact_diag_task");
  DiagnosticStatus got;
  bool seen = false;
  std::mutex m;
  auto sub =
      r.client_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/diagnostics", rclcpp::QoS(10).reliable(),
          [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr a) {
            for (const auto &s : a->status)
              if (has(s.name, "Contact detection")) {
                std::lock_guard<std::mutex> l(m);
                got = s;
                seen = true;
              }
          });
  for (int i = 0; i < 150 && !seen; ++i)
    rclcpp::spin_some(r.client_node), std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(seen);
  EXPECT_EQ(got.level, DiagnosticStatus::OK);
  EXPECT_EQ(got.hardware_id, "kinova_gen3");
}
