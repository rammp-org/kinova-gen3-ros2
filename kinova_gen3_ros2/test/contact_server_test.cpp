// /set_contact_thresholds, the "Contact detection" diagnostics and the
// contact_debug_topic residual topic, against a fake contact sink behind core's
// real Arbiter (enforced), so the token gate is the real one. No robot.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinova_gen3_ros2/contact_server.h"
#include "kinova_lowlevel/interface/arbiter.h"
#include "fake_contact_sink.h"
#include "fake_gripper_sink.h"
#include "fake_stream_sink.h"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using kinova_gen3_ros2::ContactServer;
using Srv = ContactServer::SetContactThresholds;
using sensor_msgs::msg::JointState;

namespace {
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const std::vector<std::string> kArmJoints = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7"};

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

  explicit Rig(const std::string &name, std::string detection = "enforce",
               bool debug_topic = false) {
    token = arb.grant("contact_server_test").token;
    node = std::make_shared<rclcpp::Node>(name);
    client_node = std::make_shared<rclcpp::Node>(name + "_client");
    server = std::make_unique<ContactServer>(node, arb, std::move(detection),
                                             debug_topic, kArmJoints);
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

  // A request that never sets `profile` sends 0: the default profile, not the
  // most sensitive one.
  res = r.call(Srv::Request().profile, {});
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->accepted) << res->message;
  EXPECT_EQ(res->profile, Srv::Request::PROFILE_DEFAULT);
  EXPECT_EQ(to_std(res->thresholds), to_std(r.sink.profiles.normal));
  EXPECT_EQ(r.sink.sets.back().profile,
            kinova::interface::ContactProfile::kDefault);
}

// contact_debug_topic is off by default: no residual topic, and no
// /contact_monitor either.
TEST_F(ContactServerTest, DebugTopicOffByDefault) {
  Rig r("contact_debug_off");
  // The graph works: the server's own /diagnostics publisher is counted.
  EXPECT_GE(r.node->count_publishers("/diagnostics"), 1u);
  EXPECT_EQ(r.node->count_publishers("/contact_monitor/residual"), 0u);
  EXPECT_EQ(r.node->count_publishers("/contact_monitor"), 0u);
}

// With detection off there is nothing to publish, so the topic is not
// created even when asked for.
TEST_F(ContactServerTest, DebugTopicNotCreatedWithDetectionOff) {
  Rig r("contact_debug_detection_off", "off", /*debug_topic=*/true);
  EXPECT_GE(r.node->count_publishers("/diagnostics"), 1u);
  EXPECT_EQ(r.node->count_publishers("/contact_monitor/residual"), 0u);
}

// On: a JointState with the arm joint names and effort = r, on a default
// (reliable) subscription.
TEST_F(ContactServerTest, DebugTopicPublishesResidual) {
  Rig r("contact_debug_on", "enforce", /*debug_topic=*/true);
  {
    std::lock_guard<std::mutex> l(r.sink.m);
    r.sink.state.status.r << 0.1, -25, 0.3, 0.4, 0.5, 0.6, 0.7;
  }
  JointState got;
  bool seen = false;
  std::mutex m;
  auto sub = r.client_node->create_subscription<JointState>(
      "/contact_monitor/residual", 10, [&](JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> l(m);
        got = *msg;
        seen = true;
      });
  // The server's own 50 Hz timer publishes.
  for (int i = 0; i < 100 && !seen; ++i)
    rclcpp::spin_some(r.client_node), std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(seen);
  EXPECT_EQ(got.name, kArmJoints);
  EXPECT_EQ(got.effort,
            std::vector<double>({0.1, -25, 0.3, 0.4, 0.5, 0.6, 0.7}));
  EXPECT_TRUE(got.position.empty());
  EXPECT_TRUE(got.velocity.empty());
  EXPECT_NE(rclcpp::Time(got.header.stamp).nanoseconds(), 0);
}

// The trip record the diagnostics carry, with the state around it.
TEST(ContactDiagnostics, TripRecordKeyValues) {
  kinova::interface::ContactState s;
  s.enabled = true;
  s.profile = kinova::interface::ContactProfile::kLoose;
  s.status.phase = kinova::ContactPhase::kArmed;
  s.status.r << 0.1, 25, 0.3, 0.4, 0.5, 0.6, 0.7;
  s.status.theta << 24, 24, 18, 18, 6, 6, 6;
  s.status.would_trips = 3;
  s.status.bad_samples = 2;
  auto values = [](const kinova::interface::ContactState &st) {
    diagnostic_updater::DiagnosticStatusWrapper w;
    kinova_gen3_ros2::contact_diagnostics(st, "enforce", w);
    std::map<std::string, std::string> kv;
    for (const auto &v : w.values)
      kv[v.key] = v.value;
    return kv;
  };

  // No trip yet: no trip record.
  auto kv = values(s);
  EXPECT_EQ(kv["detection"], "enforce");
  EXPECT_EQ(kv["phase"], "armed");
  EXPECT_EQ(kv["profile"], "loose");
  EXPECT_EQ(kv["residual_nm"], "[0.10, 25.00, 0.30, 0.40, 0.50, 0.60, 0.70]");
  EXPECT_EQ(kv["thresholds_nm"],
            "[24.00, 24.00, 18.00, 18.00, 6.00, 6.00, 6.00]");
  EXPECT_EQ(kv["would_trips"], "3");
  EXPECT_EQ(kv["bad_samples"], "2");
  for (const auto &k :
       {"last_trip_joint", "last_trip_residual_nm", "last_trip_thresholds_nm",
        "last_trip_q_rad", "last_trip_qd_rad_s"})
    EXPECT_EQ(kv.count(k), 0u) << k;

  s.status.phase = kinova::ContactPhase::kLatched;
  s.status.has_trip = true;
  s.status.last.joint = 1; // 0-based in core
  s.status.last.r << 0, 25.5, 0, 0, 0, 0, 0;
  s.status.last.theta << 16, 16, 12, 12, 4, 4, 4;
  s.status.last.q << 1, 2, 3, 4, 5, 6, -7;
  s.status.last.qd << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -0.7;
  kv = values(s);
  EXPECT_EQ(kv["last_trip_joint"], "2"); // 1-based
  EXPECT_EQ(kv["last_trip_residual_nm"],
            "[0.00, 25.50, 0.00, 0.00, 0.00, 0.00, 0.00]");
  // θ at the trip, not today's.
  EXPECT_EQ(kv["last_trip_thresholds_nm"],
            "[16.00, 16.00, 12.00, 12.00, 4.00, 4.00, 4.00]");
  EXPECT_EQ(kv["last_trip_q_rad"],
            "[1.00, 2.00, 3.00, 4.00, 5.00, 6.00, -7.00]");
  EXPECT_EQ(kv["last_trip_qd_rad_s"],
            "[0.10, 0.20, 0.30, 0.40, 0.50, 0.60, -0.70]");
}

TEST(ContactDiagnostics, DiagnosticsEachPhase) {
  using kinova::ContactPhase;
  kinova::interface::ContactState s;
  s.enabled = true;
  s.profile = kinova::interface::ContactProfile::kDefault;
  s.status.theta << 16, 16, 12, 12, 4, 4, 4;
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
  s.status.last.r << 0, 0, -12.8, 0, 0, 0, 0;
  s.status.last.theta = s.status.theta;
  w = diag(s, "enforce");
  EXPECT_EQ(w.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(w.message, "stopped on contact: joint 3, |r| = 12.80 N·m > θ = "
                       "12.00 N·m at the trip (profile now default); call "
                       "/clear_faults");
  // The profile can change while latched: θ stays the trip's, the profile is
  // today's, and the message says which is which.
  kinova::interface::ContactState relaxed = s;
  relaxed.profile = kinova::interface::ContactProfile::kLoose;
  relaxed.status.theta << 24, 24, 18, 18, 6, 6, 6;
  EXPECT_EQ(diag(relaxed, "enforce").message,
            "stopped on contact: joint 3, |r| = 12.80 N·m > θ = 12.00 N·m at "
            "the trip (profile now loose); call /clear_faults");
  const auto trip_r =
      std::find_if(w.values.begin(), w.values.end(), [](const auto &kv) {
        return kv.key == "last_trip_residual_nm";
      });
  ASSERT_NE(trip_r, w.values.end());
  EXPECT_NE(trip_r->value.find("-12.8"), std::string::npos)
      << trip_r->value; // signed

  kinova::interface::ContactState obs = s;
  obs.status.detection = kinova::ContactDetection::kObserve;
  obs.status.phase = ContactPhase::kArmed;
  obs.status.would_trips = 3;
  w = diag(obs, "observe");
  EXPECT_EQ(w.level, DiagnosticStatus::WARN);
  EXPECT_EQ(w.message,
            "observe mode: not stopping the arm; 3 would-be trips, last on "
            "joint 3");
  obs.status.would_trips = 0;
  obs.status.has_trip = false;
  EXPECT_EQ(diag(obs, "observe").message,
            "observe mode: not stopping the arm; 0 would-be trips");

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
