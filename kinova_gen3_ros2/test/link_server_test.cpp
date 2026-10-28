// /clear_faults and the "Arm link" diagnostics, against a real core ArmLink
// over SimTransport (no robot).
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "kinova_gen3_ros2/link_server.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace {
kinova::ExecutorConfig fast_exec() {
  kinova::ExecutorConfig c{
      10000.0, kinova::Pacing::kSleepSpin, {0, -1, false, false}};
  c.exit_on_fault = true;
  return c;
}
kinova::ArmLinkConfig fast_link() {
  kinova::ArmLinkConfig c;
  c.retry_period_s = 0.01;
  c.fault_poll_hz = 1000.0;
  c.health_period_s = 0.005;
  c.clear_settle_s = 0.05;
  return c;
}

// SimTransport -> RtExecutor -> ArmLink on its own thread, plus the server.
struct Rig {
  kinova::SimTransport sim{kinova::JointFeedback{}};
  kinova::SampleRing ring{1u << 12};
  kinova::RtExecutor exec{sim, ring, fast_exec()};
  kinova::ArmLink link{sim, exec, nullptr, fast_link()};
  std::atomic<bool> stop{false};
  std::thread th;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<kinova_gen3_ros2::LinkServer> server;
  rclcpp::executors::MultiThreadedExecutor ex;
  std::thread spin;

  explicit Rig(const std::string &name) {
    node = std::make_shared<rclcpp::Node>(name);
    server = std::make_unique<kinova_gen3_ros2::LinkServer>(node, link, "sim");
    ex.add_node(node);
    spin = std::thread([this] { ex.spin(); });
  }
  void start() {
    th = std::thread([this] { link.run(stop); });
  }
  bool wait_state(kinova::LinkState s) {
    for (int i = 0; i < 500; ++i) {
      if (link.status().state == s)
        return true;
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }
  // Calls /clear_faults from a separate client node, so the server's spinning
  // executor is the only one touching `node`.
  std::shared_ptr<Trigger::Response> clear() {
    auto cn = std::make_shared<rclcpp::Node>(node->get_name() +
                                             std::string("_client"));
    auto client = cn->create_client<Trigger>("clear_faults");
    if (!client->wait_for_service(5s)) {
      ADD_FAILURE() << "clear_faults never appeared";
      return nullptr;
    }
    auto fut = client->async_send_request(std::make_shared<Trigger::Request>());
    if (rclcpp::spin_until_future_complete(cn, fut, 15s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "no response within 15 s";
      return nullptr;
    }
    return fut.get();
  }
  ~Rig() {
    stop.store(true);
    if (th.joinable())
      th.join();
    ex.cancel();
    if (spin.joinable())
      spin.join();
  }
};
} // namespace

class LinkServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

TEST_F(LinkServerTest, ClearWhenNotFaultedIsRefused) {
  Rig r("link_clear_running");
  r.start();
  ASSERT_TRUE(r.wait_state(kinova::LinkState::kRunning));
  const auto res = r.clear();
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "not faulted (running)");
}

TEST_F(LinkServerTest, ClearRecoversAFaultedArm) {
  Rig r("link_clear_ok");
  r.sim.set_arm_fault(true);
  r.start();
  ASSERT_TRUE(r.wait_state(kinova::LinkState::kFaulted));
  const auto res = r.clear();
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_EQ(res->message, "running");
  EXPECT_EQ(r.link.status().state, kinova::LinkState::kRunning);
}

TEST_F(LinkServerTest, ClearThatDoesNotTakeSaysWhy) {
  Rig r("link_clear_sticky");
  r.sim.set_arm_fault(true);
  r.sim.set_fault_sticky(true);
  r.start();
  ASSERT_TRUE(r.wait_state(kinova::LinkState::kFaulted));
  const auto res = r.clear();
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "faulted: fault persists after clear");
}

TEST(LinkDiagnostics, EachStateMapsToItsLevel) {
  kinova::LinkStatus s;
  diagnostic_updater::DiagnosticStatusWrapper run, fault, conn;
  s.state = kinova::LinkState::kRunning;
  kinova_gen3_ros2::link_diagnostics(s, "192.168.1.10", run);
  EXPECT_EQ(run.level, DiagnosticStatus::OK);

  s.state = kinova::LinkState::kFaulted;
  s.reason = "the arm reports a fault";
  s.faults = 2;
  kinova_gen3_ros2::link_diagnostics(s, "192.168.1.10", fault);
  EXPECT_EQ(fault.level, DiagnosticStatus::ERROR);
  EXPECT_NE(fault.message.find("the arm reports a fault"), std::string::npos);

  s.state = kinova::LinkState::kConnecting;
  s.reason = "no route to host";
  kinova_gen3_ros2::link_diagnostics(s, "192.168.1.10", conn);
  EXPECT_EQ(conn.level, DiagnosticStatus::STALE);
  EXPECT_EQ(conn.message,
            "waiting for the arm at 192.168.1.10: no route to host");
  bool saw_faults = false, saw_state = false;
  for (const auto &kv : conn.values) {
    saw_faults |= kv.key == "faults" && kv.value == "2";
    saw_state |= kv.key == "link_state" && kv.value == "connecting";
  }
  EXPECT_TRUE(saw_faults);
  EXPECT_TRUE(saw_state);
}
