// kinova_gen3_ros2/src/arbitration_server.cpp
#include "kinova_gen3_ros2/arbitration_server.h"
#include <chrono>
#include <functional>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_gen3_ros2 {
using namespace kinova::interface;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {
// Compare the PAYLOAD only. The header stamp changes every poll, so including
// it would make "on change" mean "at 10 Hz forever".
bool same(const kinova_gen3_interfaces::msg::ControlStatus &a,
          const kinova_gen3_interfaces::msg::ControlStatus &b) {
  return a.arbitration_enabled == b.arbitration_enabled &&
         a.estopped == b.estopped && a.owned == b.owned &&
         a.owner_id == b.owner_id && a.generation == b.generation &&
         a.rejected_count == b.rejected_count;
}
} // namespace

ArbitrationServer::ArbitrationServer(rclcpp::Node::SharedPtr node,
                                     ArbitrationSink &arb,
                                     const std::string &hardware_id,
                                     double estop_clear_max_age_s)
    : node_(node), arb_(arb), estop_clear_max_age_s_(estop_clear_max_age_s) {
  acquire_srv_ = node_->create_service<AcquireControl>(
      "acquire_control",
      std::bind(&ArbitrationServer::on_acquire, this, _1, _2));
  release_srv_ = node_->create_service<ReleaseControl>(
      "release_control",
      std::bind(&ArbitrationServer::on_release, this, _1, _2));
  revoke_srv_ = node_->create_service<RevokeControl>(
      "revoke_control", std::bind(&ArbitrationServer::on_revoke, this, _1, _2));

  // LATCHED: a client that starts late or reconnects must learn
  // owner/generation/ estopped immediately rather than waiting for the next
  // change. Safe because we are the publisher -- an offered transient_local is
  // compatible with volatile subscribers.
  status_pub_ = node_->create_publisher<ControlStatus>(
      "control_status", rclcpp::QoS(1).reliable().transient_local());

  // /estop gets its OWN mutually-exclusive callback group. Arbiter::estop() is
  // deliberately lock-free so it cannot queue behind a delegated call holding
  // the arbiter mutex (today a cuRobo round-trip; after the streaming tier, a
  // 250 ms mode settle). Sharing a group with those would rebuild that stall in
  // ROS.
  estop_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = estop_group_;
  // VOLATILE, deliberately. A transient_local SUBSCRIPTION is incompatible with
  // a volatile publisher -- which is what `ros2 topic pub` and rqt produce --
  // so requesting durability here would make an operator's e-stop silently fail
  // to connect. Leading '/' keeps the topic global rather than node-namespaced.
  estop_sub_ = node_->create_subscription<EStop>(
      "/estop", rclcpp::QoS(10).reliable(),
      std::bind(&ArbitrationServer::on_estop, this, _1), opts);

  status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), [this] { publish_status_if_changed(); });

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID(hardware_id);
  updater_->add("Arbitration", this, &ArbitrationServer::diagnostics);

  publish_status_if_changed(); // seed the latched topic
}

void ArbitrationServer::forget_token() {
  std::lock_guard<std::mutex> l(m_);
  have_retained_ = false;
  retained_token_ = Token{};
}

void ArbitrationServer::on_acquire(
    const std::shared_ptr<AcquireControl::Request> req,
    std::shared_ptr<AcquireControl::Response> resp) {
  // Read BEFORE granting: grant() SEIZES. If someone already holds the arm they
  // are about to be dispossessed and their in-flight goal settled -9. In a
  // high-trust system an unexpected seizure is exactly the mistake this tier
  // exists to catch, so it is loud rather than inferable from a generation
  // counter nobody watched.
  const ArbitrationStatus before = arb_.status();
  const GrantResult r = arb_.grant(req->owner_id);
  if (r.accepted && before.owned && before.owner_id != req->owner_id) {
    RCLCPP_WARN(node_->get_logger(),
                "control SEIZED: '%s' took the arm from '%s' (generation %llu "
                "-> %llu); "
                "any motion the previous owner had running is being halted",
                req->owner_id.c_str(), before.owner_id.c_str(),
                static_cast<unsigned long long>(before.generation),
                static_cast<unsigned long long>(r.generation));
  }
  resp->accepted = r.accepted;
  resp->token = r.token; // Token IS std::array<uint8_t,16>; direct assign
  resp->generation = r.generation;
  resp->message = r.message;
  {
    std::lock_guard<std::mutex> l(m_);
    have_retained_ = r.accepted;
    retained_token_ = r.accepted ? r.token : Token{};
  }
  publish_status_if_changed();
}

void ArbitrationServer::on_release(
    const std::shared_ptr<ReleaseControl::Request> req,
    std::shared_ptr<ReleaseControl::Response> resp) {
  // ArbitrationStatus deliberately does NOT carry the token -- publishing a
  // capability on a status topic would defeat it -- so the check is against the
  // token this class minted. Sound because ArbitrationServer is the only caller
  // of grant/revoke/estop.
  bool ok = false;
  {
    std::lock_guard<std::mutex> l(m_);
    ok = have_retained_ && req->token == retained_token_;
  }
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

void ArbitrationServer::on_revoke(
    const std::shared_ptr<RevokeControl::Request> req,
    std::shared_ptr<RevokeControl::Response> resp) {
  // No token: this is the recovery path for a crashed owner, and ownership has
  // no lease.
  RCLCPP_WARN(node_->get_logger(), "operator revoke: %s",
              req->reason.empty() ? "(no reason given)" : req->reason.c_str());
  arb_.revoke();
  forget_token();
  resp->revoked = true;
  resp->message = "";
  publish_status_if_changed();
}

void ArbitrationServer::on_estop(const EStop::SharedPtr msg) {
  if (msg->engaged) {
    // NEVER age-checked. Refusing an old stop because its clock looked wrong is
    // precisely the failure we must not build: a stale stop is still honoured.
    RCLCPP_WARN(
        node_->get_logger(), "E-STOP engaged by '%s': %s", msg->source.c_str(),
        msg->reason.empty() ? "(no reason given)" : msg->reason.c_str());
    arb_.estop();
    forget_token(); // estop() clears ownership inside the Arbiter too
    publish_status_if_changed();
    return;
  }

  // Clearing IS age-checked: it re-enables a stopped arm, so a `ros2 bag`
  // replay or a message delayed behind a network hiccup must not do it
  // silently.
  const bool unstamped =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
  if (unstamped) {
    // Accepted anyway: `ros2 topic pub` leaves the stamp at zero, and an e-stop
    // control that cannot be driven from the CLI is worse than one that can be
    // replayed.
    RCLCPP_WARN(
        node_->get_logger(),
        "unstamped /estop clear from '%s' accepted; stamp it to enable the "
        "staleness guard",
        msg->source.c_str());
  } else if (estop_clear_max_age_s_ > 0.0) {
    // Construct with the node's clock type: subtracting rclcpp::Time values of
    // different clock types throws, and the default for a raw stamp is not
    // guaranteed to match node_->now() under use_sim_time.
    const rclcpp::Time stamp(msg->header.stamp,
                             node_->get_clock()->get_clock_type());
    const double age = (node_->now() - stamp).seconds();
    if (age > estop_clear_max_age_s_) {
      RCLCPP_WARN(
          node_->get_logger(),
          "IGNORING stale /estop clear from '%s' (age %.2fs > %.2fs); the arm "
          "stays stopped",
          msg->source.c_str(), age, estop_clear_max_age_s_);
      return;
    }
  }
  RCLCPP_WARN(
      node_->get_logger(),
      "e-stop CLEARED by '%s'; the arm has no owner until someone re-acquires "
      "control",
      msg->source.c_str());
  arb_.estop_clear();
  publish_status_if_changed();
}

void ArbitrationServer::publish_status_if_changed() {
  const ArbitrationStatus s = arb_.status();
  ControlStatus m;
  m.header.stamp = node_->now();
  m.arbitration_enabled = (s.mode == ArbitrationMode::kEnforced);
  m.estopped = s.estopped;
  m.owned = s.owned;
  m.owner_id = s.owner_id;
  m.generation = s.generation;
  m.rejected_count = s.rejected_count;
  {
    std::lock_guard<std::mutex> l(m_);
    if (last_published_ && same(*last_published_, m))
      return;
    last_published_ = m;
  }
  status_pub_->publish(m);
}

void ArbitrationServer::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const ArbitrationStatus s = arb_.status();
  const bool enforced = (s.mode == ArbitrationMode::kEnforced);
  if (s.estopped)
    stat.summary(DiagnosticStatus::ERROR, "E-STOPPED");
  else if (enforced && !s.owned)
    stat.summary(DiagnosticStatus::WARN, "no owner");
  else
    stat.summary(DiagnosticStatus::OK, "OK");
  // REP 107 names KeyValues as the place for "error counts, and information on
  // latest errors or timeouts".
  stat.add("arbitration_mode", enforced ? "enforced" : "disabled");
  stat.add("estopped", s.estopped);
  stat.add("owned", s.owned);
  stat.add("owner_id", s.owner_id.empty() ? std::string("(none)") : s.owner_id);
  stat.add("generation", static_cast<int>(s.generation));
  stat.add("rejected_count", static_cast<int>(s.rejected_count));
}
} // namespace kinova_gen3_ros2
