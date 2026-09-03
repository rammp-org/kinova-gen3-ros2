#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_ros2/stream_server.h"
#include "kinova_lowlevel/interface/streaming_session.h"
#include "fake_stream_sink.h"
using namespace std::chrono_literals;

namespace {
class SpinThread {
public:
  explicit SpinThread(rclcpp::Executor &ex)
      : ex_(ex), t_([&ex] { ex.spin(); }) {}
  ~SpinThread() {
    ex_.cancel();
    if (t_.joinable())
      t_.join();
  }
  SpinThread(const SpinThread &) = delete;
  SpinThread &operator=(const SpinThread &) = delete;

private:
  rclcpp::Executor &ex_;
  std::thread t_;
};

kinova::interface::Token mktoken(uint8_t x) {
  kinova::interface::Token t{};
  t[0] = x;
  return t;
}

class StreamServerTest : public ::testing::Test {
protected:
  // The executor must NOT be a member: members are constructed before SetUp
  // runs, and building one before rclcpp::init throws "context argument is
  // null".
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("stream_server_test");
    server_ = std::make_unique<kinova_gen3_ros2::StreamServer>(node_, sink_);
    ex_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    ex_->add_node(node_);
  }
  void TearDown() override {
    ex_.reset();
    server_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  template <class SrvT>
  typename SrvT::Response::SharedPtr
  call(const std::string &name, typename SrvT::Request::SharedPtr req) {
    auto client = node_->create_client<SrvT>(name);
    SpinThread spin(*ex_);
    if (!client->wait_for_service(3s))
      return nullptr;
    auto fut = client->async_send_request(req);
    if (fut.wait_for(3s) != std::future_status::ready)
      return nullptr;
    return fut.get();
  }

  template <class MsgT>
  void publish_setpoint(const std::string &topic, const MsgT &m) {
    auto pub = node_->create_publisher<MsgT>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    SpinThread spin(*ex_);
    for (int i = 0; i < 200 && pub->get_subscription_count() == 0; ++i)
      std::this_thread::sleep_for(10ms);
    pub->publish(m);
    std::this_thread::sleep_for(300ms);
  }

  std::vector<kinova_gen3_interfaces::msg::StreamStatus>
  collect_status(std::chrono::milliseconds dwell) {
    std::vector<kinova_gen3_interfaces::msg::StreamStatus> got;
    std::mutex gm;
    auto sub =
        node_->create_subscription<kinova_gen3_interfaces::msg::StreamStatus>(
            "stream_status", rclcpp::QoS(10).reliable().transient_local(),
            [&got,
             &gm](kinova_gen3_interfaces::msg::StreamStatus::SharedPtr m) {
              std::lock_guard<std::mutex> l(gm);
              got.push_back(*m);
            });
    {
      SpinThread spin(*ex_);
      std::this_thread::sleep_for(dwell);
    }
    std::lock_guard<std::mutex> l(gm);
    return got;
  }

  rclcpp::Node::SharedPtr node_;
  FakeStreamSink sink_;
  std::unique_ptr<kinova_gen3_ros2::StreamServer> server_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> ex_;
};
} // namespace

// -------------------------------------------------------------------
// /list_controllers

TEST_F(StreamServerTest, ListsEveryControllerWithItsChannels) {
  using Srv = kinova_gen3_interfaces::srv::ListControllers;
  auto resp = call<Srv>("list_controllers", std::make_shared<Srv::Request>());
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->controllers.size(), 8u);

  auto find = [&](const std::string &n)
      -> const kinova_gen3_interfaces::msg::ControllerCapability * {
    for (const auto &c : resp->controllers)
      if (c.name == n)
        return &c;
    return nullptr;
  };
  const auto *imp = find("joint_impedance");
  ASSERT_NE(imp, nullptr);
  EXPECT_TRUE(imp->available);
  ASSERT_EQ(imp->channels.size(), 1u);
  EXPECT_EQ(imp->channels[0], "joint_position");

  // Availability is computed from core's pair_supported(), so these track core
  // rather than a hand-maintained list. They were false until core landed
  // JointVelocityMode (core PR #32) and became true here with no change to this
  // repo beyond the expectation below -- which is the property the registry
  // exists to have.
  const auto *vel = find("joint_velocity");
  ASSERT_NE(vel, nullptr);
  EXPECT_TRUE(vel->available);
  const auto *twist = find("ee_twist");
  ASSERT_NE(twist, nullptr);
  EXPECT_TRUE(twist->available);

  // EE pose into POSITION mode -- distinct from ee_pose_impedance. Position
  // mode has no compliance, so the servo chases the pose at full authority.
  const auto *stiff = find("ee_pose_position");
  ASSERT_NE(stiff, nullptr);
  EXPECT_TRUE(stiff->available);
  ASSERT_EQ(stiff->channels.size(), 1u);
  EXPECT_EQ(stiff->channels[0], "pose");

  // Unavailable for two independent reasons: core has no kEeWrench, AND a
  // two-channel controller needs multi-channel sessions.
  const auto *cart = find("cartesian_impedance");
  ASSERT_NE(cart, nullptr);
  EXPECT_FALSE(cart->available);
  EXPECT_EQ(cart->channels.size(), 2u);
}

// ------------------------------------------------------------------- open /
// close

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
  EXPECT_EQ(resp->channels[0], "joint_position"); // the driver tells you where

  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "open");
  EXPECT_EQ(sink_.last_open.kind,
            kinova::interface::SetpointKind::kJointPosition);
  EXPECT_EQ(sink_.last_open.control_mode,
            kinova::interface::ControlModeKind::kImpedance);
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

// cartesian_impedance is unavailable, and core has no kind for it at all -- so
// the rejection has to originate here, not in pair_supported().
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
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "close");
  EXPECT_EQ(sink_.last_token, mktoken(0xCD));
}

// ------------------------------------------------------------------- setpoint
// routing

TEST_F(StreamServerTest, JointTopicsRouteToTheirOwnSinkMethod) {
  kinova_gen3_interfaces::msg::JointSetpoint m;
  m.values = {0.1, 0, 0, 0, 0, 0, 0};
  m.token = mktoken(0xAB);

  publish_setpoint("/setpoint/joint_position", m);
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "joint_position");
  EXPECT_EQ(sink_.last_token, mktoken(0xAB)); // token survives the hop
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
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "pose");
  EXPECT_EQ(sink_.last_token, mktoken(0x11));

  kinova_gen3_interfaces::msg::TwistSetpoint t;
  t.twist.linear.x = 0.05;
  t.token = mktoken(0x22);
  publish_setpoint("/setpoint/twist", t);
  EXPECT_EQ(sink_.log().back(), "twist");
  EXPECT_EQ(sink_.last_token, mktoken(0x22));
}

// Core has no on_setpoint_wrench, so there is nowhere to route this. The topic
// exists so the surface is complete; this guards against someone later wiring
// wrench into the wrong on_setpoint_* method.
TEST_F(StreamServerTest, WrenchIsDroppedBecauseCoreHasNoSinkForIt) {
  kinova_gen3_interfaces::msg::WrenchSetpoint w;
  w.wrench.force.z = 5.0;
  w.token = mktoken(0x33);
  publish_setpoint("/setpoint/wrench", w);
  EXPECT_TRUE(sink_.log().empty());
}

// -------------------------------------------------------------------
// /stream_status

// open/timeout/rejected_count come from core, NOT from what this node
// remembers.
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

// The case that motivated core PR #31: we opened a session, core expired it,
// and the status must follow core rather than our own record.
TEST_F(StreamServerTest, StatusFollowsCoreWhenTheSessionExpires) {
  using Srv = kinova_gen3_interfaces::srv::OpenStream;
  auto req = std::make_shared<Srv::Request>();
  req->controller = "joint_impedance";
  req->timeout_s = 0.1;
  sink_.status.open = true;
  ASSERT_NE(call<Srv>("open_stream", req), nullptr);
  {
    const auto got = collect_status(300ms);
    ASSERT_FALSE(got.empty());
    EXPECT_TRUE(got.back().open);
    EXPECT_EQ(got.back().controller, "joint_impedance");
  }

  sink_.status.open = false; // core tore it down; nobody told us
  const auto got = collect_status(400ms);
  ASSERT_FALSE(got.empty());
  EXPECT_FALSE(got.back().open);
  EXPECT_EQ(got.back().controller, ""); // our label is dropped with it
}

TEST_F(StreamServerTest, StatusIsNotRepublishedWhileUnchanged) {
  const auto got = collect_status(700ms);
  EXPECT_LE(got.size(), 1u)
      << "status republished " << got.size() << " times with nothing changing";
}

// The registry must stay in step with core in BOTH directions. Availability is
// already computed from pair_supported(), so a pair core RETIRES turns a row
// unavailable on its own -- but a pair core ADDS is invisible until someone
// adds a row, and the surface silently fails to expose a capability the driver
// has.
//
// Core PR #32 is exactly that case: it made kEePose x kPosition supported, and
// nothing in this repo would have noticed. This test fails naming the missing
// pair instead.
TEST(StreamServerRegistry, EverySupportedCorePairHasAController) {
  using kinova::interface::SetpointKind;
  using kinova::interface::ControlModeKind;
  const SetpointKind kinds[] = {
      SetpointKind::kJointPosition, SetpointKind::kEePose,
      SetpointKind::kJointVelocity, SetpointKind::kEeTwist,
      SetpointKind::kJointTorque};
  const ControlModeKind modes[] = {
      ControlModeKind::kPosition, ControlModeKind::kImpedance,
      ControlModeKind::kVelocity, ControlModeKind::kTorque};

  for (auto k : kinds) {
    for (auto m : modes) {
      if (!kinova::interface::pair_supported(k, m))
        continue;
      bool covered = false;
      for (const auto &r : kinova_gen3_ros2::StreamServer::registry())
        if (r.core_backed && r.kind == k && r.mode == m) {
          covered = true;
          break;
        }
      EXPECT_TRUE(covered)
          << "core supports (kind=" << static_cast<int>(k)
          << ", mode=" << static_cast<int>(m)
          << ") but no controller in StreamServer::registry() exposes it";
    }
  }
}

// The other direction: a row claiming to be core-backed must name a pair core
// actually supports, or /list_controllers advertises something open_stream will
// refuse.
TEST(StreamServerRegistry, EveryAvailableControllerMapsToASupportedPair) {
  for (const auto &r : kinova_gen3_ros2::StreamServer::registry()) {
    if (!kinova_gen3_ros2::StreamServer::available(r))
      continue;
    EXPECT_TRUE(kinova::interface::pair_supported(r.kind, r.mode))
        << "controller '" << r.name
        << "' reports available but its pair is unsupported";
  }
}
