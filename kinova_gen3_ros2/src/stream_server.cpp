// kinova_gen3_ros2/src/stream_server.cpp
#include "kinova_gen3_ros2/stream_server.h"
#include <array>
#include <chrono>
#include <functional>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kinova_lowlevel/interface/streaming_session.h" // pair_supported
namespace kinova_gen3_ros2 {
using namespace kinova::interface;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {
// Setpoint payload conversions. They live here rather than in message_mapping
// because only the streaming tier converts these; message_mapping is the action
// tier's seam.
kinova::JointVec to_joint_vec(const std::array<double, 7> &a) {
  kinova::JointVec v;
  for (int i = 0; i < kinova::kNumJoints; ++i)
    v[i] = a[i];
  return v;
}
kinova::Pose to_pose(const geometry_msgs::msg::Pose &p) {
  kinova::Pose out;
  out.p = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  out.R = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
                             p.orientation.z);
  return out;
}
kinova::Vector6 to_vector6(const geometry_msgs::msg::Twist &t) {
  kinova::Vector6 v; // core carries this as [linear; angular]
  v << t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y,
      t.angular.z;
  return v;
}
// Compare the PAYLOAD only -- the header stamp changes every poll, so including
// it would make "on change" mean "at 10 Hz forever".
bool same(const kinova_gen3_interfaces::msg::StreamStatus &a,
          const kinova_gen3_interfaces::msg::StreamStatus &b) {
  return a.open == b.open && a.controller == b.controller &&
         a.channels == b.channels && a.timeout_s == b.timeout_s &&
         a.rejected_count == b.rejected_count;
}
} // namespace

const std::vector<StreamServer::ControllerRow> &StreamServer::registry() {
  // The ONLY place core's (kind, mode) pair is collapsed into a controller
  // name. cartesian_impedance is core_backed=false: core has no kEeWrench, so
  // there is no pair to ask about -- and it needs two channels besides.
  static const std::vector<ControllerRow> kRows = {
      {"joint_position",
       {"joint_position"},
       true,
       SetpointKind::kJointPosition,
       ControlModeKind::kPosition},
      {"joint_impedance",
       {"joint_position"},
       true,
       SetpointKind::kJointPosition,
       ControlModeKind::kImpedance},
      {"ee_pose_impedance",
       {"pose"},
       true,
       SetpointKind::kEePose,
       ControlModeKind::kImpedance},
      {"ee_pose_position",
       {"pose"},
       true,
       SetpointKind::kEePose,
       ControlModeKind::kPosition},
      {"joint_torque",
       {"joint_torque"},
       true,
       SetpointKind::kJointTorque,
       ControlModeKind::kTorque},
      {"joint_velocity",
       {"joint_velocity"},
       true,
       SetpointKind::kJointVelocity,
       ControlModeKind::kVelocity},
      {"ee_twist",
       {"twist"},
       true,
       SetpointKind::kEeTwist,
       ControlModeKind::kVelocity},
      {"cartesian_impedance",
       {"pose", "wrench"},
       false,
       SetpointKind::kEePose,
       ControlModeKind::kImpedance},
  };
  return kRows;
}

bool StreamServer::available(const ControllerRow &r) {
  // Asking core rather than declaring: when JointVelocityMode lands, these rows
  // light up with no change here.
  return r.core_backed && r.channels.size() == 1 &&
         pair_supported(r.kind, r.mode);
}

StreamServer::StreamServer(rclcpp::Node::SharedPtr node, StreamSink &sink)
    : node_(node), sink_(sink) {
  // on_stream_open sleeps mode_settle_s (250 ms) holding the arbiter mutex, so
  // session control gets its own group -- never /estop's, and never the
  // setpoints'.
  session_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  // MutuallyExclusive: one logical writer, matching core's single-writer double
  // buffer. Separate from session_group_ so setpoints do not queue behind a 250
  // ms open.
  setpoint_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sp_opts;
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

  // Best-effort, KeepLast(1): core's setpoints are absolute and latest-wins, so
  // a dropped intermediate is correct. Reliable-with-queue would deliver stale
  // setpoints late, which is the failure the session deadline exists to catch.
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
      "/setpoint/pose", sp_qos, std::bind(&StreamServer::on_pose, this, _1),
      sp_opts);
  twist_sub_ = node_->create_subscription<TwistSetpointMsg>(
      "/setpoint/twist", sp_qos, std::bind(&StreamServer::on_twist, this, _1),
      sp_opts);
  wrench_sub_ = node_->create_subscription<WrenchSetpointMsg>(
      "/setpoint/wrench", sp_qos, std::bind(&StreamServer::on_wrench, this, _1),
      sp_opts);

  status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), [this] { publish_status_if_changed(); });
  publish_status_if_changed(); // seed the latched topic
}

void StreamServer::on_list(const std::shared_ptr<ListControllers::Request>,
                           std::shared_ptr<ListControllers::Response> resp) {
  for (const auto &r : registry()) {
    kinova_gen3_interfaces::msg::ControllerCapability c;
    c.name = r.name;
    c.channels = r.channels;
    c.available = available(r);
    resp->controllers.push_back(c);
  }
}

void StreamServer::on_open(const std::shared_ptr<OpenStream::Request> req,
                           std::shared_ptr<OpenStream::Response> resp) {
  const ControllerRow *row = nullptr;
  for (const auto &r : registry())
    if (r.name == req->controller) {
      row = &r;
      break;
    }

  // Two rejections originate HERE rather than in core: an unknown name (purely
  // this layer's vocabulary) and an unavailable controller (core may have no
  // kind for it at all, so there is nothing to ask pair_supported about).
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
  const StreamOpenResult res =
      sink_.on_stream_open(r); // blocks the mode settle

  resp->accepted = res.accepted;
  resp->error_code = res.error_code;
  resp->message = res.message;
  if (res.accepted) {
    resp->channels = row->channels; // the driver names the topics
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
  {
    std::lock_guard<std::mutex> l(m_);
    open_controller_.clear();
  }
  resp->closed = true;
  resp->message = "";
  publish_status_if_changed();
}

void StreamServer::on_joint_position(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s;
  s.values = to_joint_vec(m->values);
  s.token = m->token;
  sink_.on_setpoint_joint_position(s);
}
void StreamServer::on_joint_velocity(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s;
  s.values = to_joint_vec(m->values);
  s.token = m->token;
  sink_.on_setpoint_joint_velocity(s);
}
void StreamServer::on_joint_torque(const JointSetpointMsg::SharedPtr m) {
  JointSetpoint s;
  s.values = to_joint_vec(m->values);
  s.token = m->token;
  sink_.on_setpoint_joint_torque(s);
}
void StreamServer::on_pose(const PoseSetpointMsg::SharedPtr m) {
  PoseSetpoint s;
  s.pose = to_pose(m->pose);
  s.token = m->token;
  sink_.on_setpoint_pose(s);
}
void StreamServer::on_twist(const TwistSetpointMsg::SharedPtr m) {
  TwistSetpoint s;
  s.twist = to_vector6(m->twist);
  s.token = m->token;
  sink_.on_setpoint_twist(s);
}
void StreamServer::on_wrench(const WrenchSetpointMsg::SharedPtr) {
  // Core has no SetpointKind::kEeWrench and no on_setpoint_wrench, so there is
  // nothing to delegate to. Throttled because a client streaming wrench will
  // send at rate.
  RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "/setpoint/wrench: no controller in this driver version consumes "
      "wrench setpoints; dropping");
}

void StreamServer::publish_status_if_changed() {
  // open/timeout/rejected_count come from CORE, not from what we remember
  // opening. Without on_query_stream (core PR #31) this could only report the
  // session we think we have, which goes stale the moment the sampler expires
  // one.
  const StreamStatus s = sink_.on_query_stream();
  StreamStatusMsg m;
  m.header.stamp = node_->now();
  m.open = s.open;
  m.timeout_s = s.timeout_s;
  m.rejected_count = s.rejected_count;
  {
    std::lock_guard<std::mutex> l(m_);
    // Core owns whether a session is open; `controller` is only our label for
    // its (kind, mode), so it is dropped as soon as core says closed.
    if (!s.open)
      open_controller_.clear();
    m.controller = open_controller_;
    for (const auto &r : registry())
      if (r.name == open_controller_) {
        m.channels = r.channels;
        break;
      }
    if (last_published_ && same(*last_published_, m))
      return;
    last_published_ = m;
  }
  status_pub_->publish(m);
}
} // namespace kinova_gen3_ros2
