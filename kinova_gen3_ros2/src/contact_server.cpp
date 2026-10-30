#include "kinova_gen3_ros2/contact_server.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_gen3_ros2 {
using kinova::ContactDetection;
using kinova::ContactPhase;
using kinova::interface::ContactProfile;
using kinova::interface::ContactState;
using Req = ContactServer::SetContactThresholds::Request;

// The wire profile bytes are core's enum values.
static_assert(static_cast<int>(ContactProfile::kDefault) ==
              Req::PROFILE_DEFAULT);
static_assert(static_cast<int>(ContactProfile::kSensitive) ==
              Req::PROFILE_SENSITIVE);
static_assert(static_cast<int>(ContactProfile::kLoose) == Req::PROFILE_LOOSE);
static_assert(static_cast<int>(ContactProfile::kCustom) == Req::PROFILE_CUSTOM);

namespace {
std::string join(const kinova::JointVec &v) {
  std::string s;
  char b[32];
  for (int i = 0; i < kinova::kNumJoints; ++i) {
    std::snprintf(b, sizeof b, "%s%.2f", i ? ", " : "", v[i]);
    s += b;
  }
  return "[" + s + "]";
}
template <typename Arr> void copy_vec(const kinova::JointVec &v, Arr &out) {
  for (int i = 0; i < kinova::kNumJoints; ++i)
    out[i] = v[i];
}
const char *phase_name(ContactPhase p) {
  switch (p) {
  case ContactPhase::kSettling:
    return "settling";
  case ContactPhase::kArmed:
    return "armed";
  case ContactPhase::kLatched:
    return "latched";
  }
  return "unknown";
}
} // namespace

ContactServer::ContactServer(rclcpp::Node::SharedPtr node,
                             kinova::interface::CommandSink &gated,
                             std::string detection, bool debug_topic,
                             std::vector<std::string> arm_joints)
    : node_(std::move(node)), sink_(gated), detection_(std::move(detection)),
      arm_joints_(std::move(arm_joints)) {
  // Its OWN mutually-exclusive group: the set goes through the Arbiter, whose
  // mutex StreamServer::on_open holds across a 250 ms mode settle, and must not
  // stall /estop or a trajectory cancel on the default group meanwhile.
  set_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  set_srv_ = node_->create_service<SetContactThresholds>(
      "set_contact_thresholds",
      std::bind(&ContactServer::on_set, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, set_group_);
  if (debug_topic && detection_ == "off") {
    RCLCPP_WARN(node_->get_logger(),
                "contact_debug_topic is on but contact detection is off: no "
                "residual to publish, /contact_monitor/residual not created");
  } else if (debug_topic) {
    // Default (reliable) QoS, so a default subscriber, Foxglove or
    // PlotJuggler matches it.
    residual_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "contact_monitor/residual", 10);
    residual_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20),
                                               [this] { publish_residual(); });
    RCLCPP_INFO(node_->get_logger(),
                "contact_debug_topic: publishing the residual r on "
                "/contact_monitor/residual (sensor_msgs/JointState, effort = r "
                "in N·m) at 50 Hz");
  }
  // Its own Updater, as LinkServer and GripperServer each own one.
  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Contact detection", this, &ContactServer::diagnostics);
}

std::string to_string(ContactProfile p) {
  switch (p) {
  case ContactProfile::kSensitive:
    return "sensitive";
  case ContactProfile::kDefault:
    return "default";
  case ContactProfile::kLoose:
    return "loose";
  case ContactProfile::kCustom:
    return "custom";
  }
  return "unknown";
}

std::optional<std::string>
to_contact_request(const Req &req,
                   kinova::interface::ContactThresholdsRequest &out) {
  switch (req.profile) {
  case Req::PROFILE_DEFAULT:
    out.profile = ContactProfile::kDefault;
    break;
  case Req::PROFILE_SENSITIVE:
    out.profile = ContactProfile::kSensitive;
    break;
  case Req::PROFILE_LOOSE:
    out.profile = ContactProfile::kLoose;
    break;
  case Req::PROFILE_CUSTOM:
    out.profile = ContactProfile::kCustom;
    break;
  default:
    return "unknown profile " + std::to_string(req.profile) +
           ": expected PROFILE_DEFAULT (0), PROFILE_SENSITIVE (1), "
           "PROFILE_LOOSE (2) or PROFILE_CUSTOM (3)";
  }
  const size_t n = req.custom.size();
  if (out.profile != ContactProfile::kCustom) {
    if (n != 0)
      return "custom must be empty for a named profile (got " +
             std::to_string(n) +
             " values); use PROFILE_CUSTOM to set exact thresholds";
  } else {
    if (n != static_cast<size_t>(kinova::kNumJoints))
      return "PROFILE_CUSTOM needs exactly " +
             std::to_string(kinova::kNumJoints) +
             " values (N·m, joints 1..7), got " + std::to_string(n);
    for (int i = 0; i < kinova::kNumJoints; ++i) {
      if (!std::isfinite(req.custom[i]))
        return "custom joint " + std::to_string(i + 1) + " threshold is " +
               std::to_string(req.custom[i]) + ": must be finite";
      out.custom[i] = req.custom[i];
    }
  }
  std::copy(req.token.begin(), req.token.end(), out.token.begin());
  return std::nullopt;
}

void ContactServer::on_set(
    const std::shared_ptr<Req> req,
    std::shared_ptr<SetContactThresholds::Response> res) {
  kinova::interface::ContactThresholdsRequest r;
  kinova::interface::ContactThresholdsResult out;
  if (auto why = to_contact_request(*req, r))
    out.message = *why;
  else
    out = sink_.on_set_contact_thresholds(r);
  res->accepted = out.accepted;
  res->message = out.message;
  if (!out.accepted) {
    // Refused: report what is active. The Arbiter's "not authorized" carries
    // no thresholds of its own, so ask.
    const ContactState s = sink_.on_query_contact();
    out.profile = s.profile;
    out.theta = s.status.theta;
    RCLCPP_WARN(node_->get_logger(), "/set_contact_thresholds refused: %s",
                out.message.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "/set_contact_thresholds: profile %s, θ = %s N·m",
                to_string(out.profile).c_str(), join(out.theta).c_str());
  }
  res->profile = static_cast<uint8_t>(out.profile);
  copy_vec(out.theta, res->thresholds);
}

void ContactServer::publish_residual() {
  const kinova::JointVec r = sink_.on_query_contact().status.r;
  sensor_msgs::msg::JointState m;
  m.header.stamp = node_->now();
  m.name = arm_joints_;
  m.effort.assign(r.data(), r.data() + r.size());
  residual_pub_->publish(m);
}

void contact_diagnostics(const ContactState &s, const std::string &detection,
                         diagnostic_updater::DiagnosticStatusWrapper &stat) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const std::string profile = to_string(s.profile);
  if (!s.enabled) {
    stat.summary(DiagnosticStatus::WARN,
                 "DISABLED (contact_detection=" + detection + ")");
  } else if (s.status.detection == ContactDetection::kObserve) {
    // The would-be trips, so a false-positive run reads off /diagnostics.
    std::string m = "observe mode: not stopping the arm; " +
                    std::to_string(s.status.would_trips) + " would-be trips";
    const int j = s.status.last.joint;
    if (s.status.has_trip && j >= 0 && j < kinova::kNumJoints)
      m += ", last on joint " + std::to_string(j + 1);
    stat.summary(DiagnosticStatus::WARN, m);
  } else if (s.status.phase == ContactPhase::kSettling) {
    stat.summary(DiagnosticStatus::WARN, "settling");
  } else if (s.status.phase == ContactPhase::kArmed) {
    stat.summary(DiagnosticStatus::OK, "armed (profile " + profile + ")");
  } else {
    const auto &t = s.status.last;
    const int j = t.joint;
    char b[160];
    // θ is the trip record's; the profile is today's, and may have changed
    // since.
    if (s.status.has_trip && j >= 0 && j < kinova::kNumJoints)
      std::snprintf(
          b, sizeof b,
          "stopped on contact: joint %d, |r| = %.2f N·m > θ = %.2f N·m at "
          "the trip (profile now %s); call /clear_faults",
          j + 1, std::abs(t.r[j]), t.theta[j], profile.c_str());
    else
      std::snprintf(b, sizeof b,
                    "stopped on contact (profile %s); call /clear_faults",
                    profile.c_str());
    stat.summary(DiagnosticStatus::ERROR, b);
  }
  stat.add("detection", detection);
  stat.add("phase", phase_name(s.status.phase));
  stat.add("profile", profile);
  stat.add("residual_nm", join(s.status.r));
  stat.add("thresholds_nm", join(s.status.theta));
  stat.add("would_trips", s.status.would_trips);
  stat.add("bad_samples", s.status.bad_samples);
  if (s.status.has_trip) {
    stat.add("last_trip_joint", s.status.last.joint + 1);
    stat.add("last_trip_residual_nm", join(s.status.last.r));
    stat.add("last_trip_thresholds_nm", join(s.status.last.theta));
    stat.add("last_trip_q_rad", join(s.status.last.q));
    stat.add("last_trip_qd_rad_s", join(s.status.last.qd));
  }
}

void ContactServer::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  contact_diagnostics(sink_.on_query_contact(), detection_, stat);
}
} // namespace kinova_gen3_ros2
