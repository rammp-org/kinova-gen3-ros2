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
bool same(const rammp_common_interfaces::msg::ControlStatus &a,
          const rammp_common_interfaces::msg::ControlStatus &b) {
  return a.arbitration_enabled == b.arbitration_enabled &&
         a.estopped == b.estopped && a.owned == b.owned &&
         a.owner_id == b.owner_id && a.generation == b.generation &&
         a.rejected_count == b.rejected_count;
}
} // namespace

ArbitrationServer::ArbitrationServer(rclcpp::Node::SharedPtr node,
                                     ArbitrationSink &arb,
                                     const std::string &hardware_id)
    : node_(node), arb_(arb) {
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
  // to connect. BEST_EFFORT for the same reason: the firmware e-stop nodes (the
  // joystick, the MIB) publish best-effort, which a reliable subscription never
  // matches, and a best-effort subscription still matches reliable publishers.
  // Each source resends every period, so a lost sample costs one period. Leading
  // '/' keeps the topic global rather than node-namespaced.
  estop_sub_ = node_->create_subscription<EStop>(
      "/estop", rclcpp::QoS(10).best_effort(),
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
  // All-zero is the srv default and means "no key supplied": the driver mints.
  const bool supplied = req->token != Token{};
  const GrantResult r =
      arb_.grant(req->owner_id,
                 supplied ? std::optional<Token>(req->token) : std::nullopt);
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
  // token granted here. Sound because ArbitrationServer is the only caller
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
  // The node LISTENS and obeys; the MIB owns the latch, the clear handshake and
  // the deadman. So no pub_seq bookkeeping, no deadman, and the stamp is never
  // read: the protocol judges time by receive time, and only the MIB does that.
  const char *reason =
      msg->reason.empty() ? "(no reason given)" : msg->reason.c_str();

  // Any state other than NOT_ENGAGED is an engage: the fail-safe reading of an
  // unknown byte.
  if (msg->state != EStop::STATE_NOT_ENGAGED) {
    // Never age- or pub_seq-checked. An engaged source keeps sending ENGAGED
    // every period until it accepts a clear, so act on a source's FIRST engage,
    // and again only if the arbiter is no longer stopped (something else
    // cleared it). Not on every repeat: each estop() makes core halt again,
    // and a halt holds at the MEASURED pose, so re-halting every period
    // re-anchors the hold 10 times a second and lets a compliant hold drift
    // with whatever pushes on it.
    const bool first = engaged_sources_.insert(msg->source).second;
    if (first)
      RCLCPP_WARN(node_->get_logger(), "E-STOP engaged by '%s': %s",
                  msg->source.c_str(), reason);
    if (first || !arb_.status().estopped) {
      arb_.estop();
      forget_token(); // estop() clears ownership inside the Arbiter too
      publish_status_if_changed();
    }
    return;
  }

  // A clear attempt never changes its SENDER's state. The MIB sends them while
  // it is still latched; counting one as the MIB going NOT_ENGAGED would
  // release the arm before the MIB confirms. Its target's own NOT_ENGAGED, and
  // then the MIB's plain NOT_ENGAGED, are what release.
  if (!msg->clear_target.empty())
    return;

  // A plain NOT_ENGAGED: this source no longer asks for the stop. From a source
  // that was not engaged it is just a heartbeat, and never ends anyone's stop.
  if (engaged_sources_.erase(msg->source) == 0)
    return;
  if (!engaged_sources_.empty()) {
    std::string others;
    for (const auto &s : engaged_sources_)
      others += (others.empty() ? "'" : ", '") + s + "'";
    RCLCPP_WARN(node_->get_logger(),
                "'%s' no longer engages the e-stop (%s); the arm stays stopped "
                "for %s",
                msg->source.c_str(), reason, others.c_str());
    return;
  }
  RCLCPP_WARN(node_->get_logger(),
              "e-stop RELEASED by '%s' (%s), the last engaged source; the arm "
              "has no owner until someone re-acquires control",
              msg->source.c_str(), reason);
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
