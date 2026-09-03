#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "kinova_arm_ros2/arbitration_server.h"
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

class ArbitrationServerTest : public ::testing::Test {
 protected:
  // The executor must NOT be a member: members are constructed before SetUp runs,
  // and building one before rclcpp::init throws "context argument is null".
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("arbitration_server_test");
    sink_.next_token = mktoken(0xAB);
    plane_ = std::make_unique<kinova_arm_ros2::ArbitrationServer>(node_, sink_, "test", 1.0);
    ex_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    ex_->add_node(node_);
  }
  void TearDown() override {
    ex_.reset();
    plane_.reset();   // drops its publishers/subscriptions while the context is alive
    node_.reset();
    rclcpp::shutdown();
  }

  // Calls a service and returns the response, or nullptr on timeout.
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

  // Publishes on /estop and spins until ArbitrationServer has had a chance to handle it.
  void publish_estop(bool engaged, const rclcpp::Time& stamp, const std::string& src) {
    auto pub = node_->create_publisher<kinova_arm_interfaces::msg::EStop>(
        "/estop", rclcpp::QoS(10).reliable());
    kinova_arm_interfaces::msg::EStop m;
    m.header.stamp = stamp;
    m.engaged = engaged;
    m.source = src;
    SpinThread spin(*ex_);
    // Wait for the subscription to match before publishing, else the message goes
    // into a void and the test races.
    for (int i = 0; i < 200 && pub->get_subscription_count() == 0; ++i)
      std::this_thread::sleep_for(10ms);
    pub->publish(m);
    std::this_thread::sleep_for(300ms);
  }

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
    { SpinThread spin(*ex_); std::this_thread::sleep_for(dwell); }
    std::lock_guard<std::mutex> l(gm);
    return got;
  }

  rclcpp::Node::SharedPtr node_;
  FakeArbitrationSink sink_;
  std::unique_ptr<kinova_arm_ros2::ArbitrationServer> plane_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> ex_;
};
}  // namespace

// ---------------------------------------------------------------- ownership services

TEST_F(ArbitrationServerTest, AcquireGrantsAndReturnsTheToken) {
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

TEST_F(ArbitrationServerTest, ReleaseWithMatchingTokenRevokes) {
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

// ArbitrationStatus deliberately does not carry the token, so ArbitrationServer checks
// against the one it minted. A stranger's token must not release someone else's arm.
TEST_F(ArbitrationServerTest, ReleaseWithWrongTokenIsRefusedAndDoesNotRevoke) {
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

TEST_F(ArbitrationServerTest, RevokeNeedsNoTokenAndAlwaysRevokes) {
  using Srv = kinova_arm_interfaces::srv::RevokeControl;
  auto req = std::make_shared<Srv::Request>();
  req->reason = "client hung";
  auto resp = call<Srv>("revoke_control", req);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->revoked);
  EXPECT_EQ(sink_.log().back(), "revoke");
}

// ---------------------------------------------------------------------- /estop policy

TEST_F(ArbitrationServerTest, EstopEngageCallsEstop) {
  publish_estop(true, node_->now(), "operator");
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "estop");
}

TEST_F(ArbitrationServerTest, FreshEstopClearCallsEstopClear) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, node_->now(), "operator");
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "estop_clear");
}

// A bag replay, or a clear delayed behind a network hiccup, must not re-enable a
// stopped arm. estop_clear_max_age_s is 1.0 in this fixture.
TEST_F(ArbitrationServerTest, StaleEstopClearIsIgnored) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, node_->now() - rclcpp::Duration::from_seconds(30.0), "replay");
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "estop");        // still stopped; no estop_clear
}

// The asymmetry, asserted so nobody later "tidies" it into a symmetric check:
// a stale STOP is still honoured. Both branches must fail toward "arm stays stopped".
TEST_F(ArbitrationServerTest, StaleEstopEngageIsStillHonoured) {
  publish_estop(true, node_->now() - rclcpp::Duration::from_seconds(30.0), "replay");
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "estop");
}

// `ros2 topic pub` leaves the stamp at zero. An e-stop control that cannot be driven
// from the CLI is worse than one that can be replayed.
TEST_F(ArbitrationServerTest, UnstampedEstopClearIsAccepted) {
  publish_estop(true, node_->now(), "operator");
  publish_estop(false, rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()), "cli");
  ASSERT_FALSE(sink_.log().empty());
  EXPECT_EQ(sink_.log().back(), "estop_clear");
}

// Engaging the e-stop clears ownership inside the Arbiter, so the token we retained
// is dead and a later release must not be honoured.
TEST_F(ArbitrationServerTest, EstopForgetsTheRetainedToken) {
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

// -------------------------------------------------------------------- /control_status

// The 10 Hz timer must NOT republish an unchanged status, or "on change" means
// "at 10 Hz forever" and the header stamp is the only thing that ever differs.
TEST_F(ArbitrationServerTest, StatusIsNotRepublishedWhileUnchanged) {
  const auto got = collect_status(700ms);
  EXPECT_LE(got.size(), 1u) << "status republished " << got.size()
                            << " times with nothing changing";
}

TEST_F(ArbitrationServerTest, StatusPublishesOnOwnershipChange) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);
  const auto got = collect_status(400ms);
  ASSERT_FALSE(got.empty());
  EXPECT_TRUE(got.back().owned);
  EXPECT_EQ(got.back().owner_id, "orchestrator");
  EXPECT_EQ(got.back().generation, 1u);
}

// A late subscriber must learn the current state without waiting for a change --
// this is what /control_status being latched buys, and what lets a reconnecting
// client discover it was dispossessed.
TEST_F(ArbitrationServerTest, LateSubscriberReceivesLatchedStatus) {
  using Acq = kinova_arm_interfaces::srv::AcquireControl;
  auto areq = std::make_shared<Acq::Request>();
  areq->owner_id = "orchestrator";
  ASSERT_NE(call<Acq>("acquire_control", areq), nullptr);
  std::this_thread::sleep_for(200ms);
  const auto got = collect_status(500ms);   // subscribes only now
  ASSERT_FALSE(got.empty());
  EXPECT_TRUE(got.back().owned);
}

// Acquiring while someone else holds the arm SEIZES it: grant() succeeds, the
// incumbent is dispossessed, and generation bumps. Asserted so the behaviour is a
// decision rather than a surprise.
TEST_F(ArbitrationServerTest, AcquireSeizesFromAnIncumbent) {
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
  const auto got = collect_status(400ms);
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.back().owner_id, "orchestrator");
}

// ----------------------------------------------------------------------- diagnostics

// REP 107: reporting is on /diagnostics using diagnostic_msgs/DiagnosticArray at 1 Hz.
// Level must be ERROR while e-stopped -- this is what a monitoring dashboard reads.
TEST_F(ArbitrationServerTest, DiagnosticsReportsErrorWhileEstopped) {
  publish_estop(true, node_->now(), "operator");

  std::vector<diagnostic_msgs::msg::DiagnosticArray> got;
  std::mutex gm;
  auto sub = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [&got, &gm](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {
        std::lock_guard<std::mutex> l(gm); got.push_back(*m);
      });
  { SpinThread spin(*ex_); std::this_thread::sleep_for(2500ms); }   // >= 2 updater ticks

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
