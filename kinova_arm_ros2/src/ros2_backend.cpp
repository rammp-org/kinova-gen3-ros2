// kinova_arm_ros2/src/ros2_backend.cpp
#include "kinova_arm_ros2/ros2_backend.h"
#include <functional>
#include <limits>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_arm_ros2/message_mapping.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_arm_ros2 {
using namespace kinova::interface;
using std::placeholders::_1; using std::placeholders::_2;

Ros2Backend::Ros2Backend(rclcpp::Node::SharedPtr node) : node_(node) {
  server_ = rclcpp_action::create_server<Action>(
      node_, "execute_joint_trajectory",
      std::bind(&Ros2Backend::handle_goal, this, _1, _2),
      std::bind(&Ros2Backend::handle_cancel, this, _1),
      std::bind(&Ros2Backend::handle_accepted, this, _1));
  state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
  // Same QoS as /joint_states: it is the same data at the same rate from the same
  // pump tick, so a subscriber matching one matches the other.
  ee_pub_ = node_->create_publisher<kinova_arm_interfaces::msg::EeState>(
      "ee_state", rclcpp::SensorDataQoS());

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Arm", this, &Ros2Backend::diagnostics);
}

rclcpp_action::GoalResponse Ros2Backend::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
  if (!sink_) return rclcpp_action::GoalResponse::REJECT;
  // Fail loud, never silent mis-mapping (CLAUDE.md): the mapping layer zero-fills a
  // short point as a memory-safety net, but a malformed goal must be REJECTED here.
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting goal: trajectory has no points");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
    const size_t n = goal->trajectory.points[i].positions.size();
    if (n != static_cast<size_t>(kinova::kNumJoints)) {
      RCLCPP_WARN(node_->get_logger(),
                  "rejecting goal: point %zu has %zu positions, expected %d",
                  i, n, kinova::kNumJoints);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  const GoalResponse r = sink_->on_trajectory_goal(to_trajectory_goal(*goal));
  return (r == GoalResponse::kAccept) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                                      : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse Ros2Backend::handle_cancel(std::shared_ptr<GoalHandle> gh) {
  if (!sink_) return rclcpp_action::CancelResponse::REJECT;
  const GoalId id = gh->get_goal_id();   // GoalUUID -> GoalId
  // Replay the token the goal was ACCEPTED with. This is FUNCTIONAL, not a security
  // measure: a ROS cancel is unauthenticated by protocol (action_msgs/CancelGoal has
  // no payload, and a zero goal id cancels everything) -- but without a valid token
  // the Arbiter refuses the cancel outright under kEnforced and the arm keeps moving.
  kinova::interface::Token token{};
  { std::lock_guard<std::mutex> l(m_);
    auto it = handles_.find(id);
    if (it != handles_.end()) token = it->second.token; }
  const CancelResponse r = sink_->on_trajectory_cancel({id, token});
  return (r == CancelResponse::kAccept) ? rclcpp_action::CancelResponse::ACCEPT
                                        : rclcpp_action::CancelResponse::REJECT;
}

void Ros2Backend::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  const TrajectoryGoal tg = to_trajectory_goal(*gh->get_goal());
  { std::lock_guard<std::mutex> l(m_); handles_[id] = Entry{gh, tg.token}; }
  sink_->on_trajectory_accepted(id, tg);
}

void Ros2Backend::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second.gh; }
  auto msg = std::make_shared<Action::Feedback>(to_feedback_msg(id, fb));
  gh->publish_feedback(msg);
}

void Ros2Backend::settle(const GoalId& id, const TrajectoryResult& r) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second.gh; handles_.erase(it); }
  auto msg = std::make_shared<Action::Result>(to_result_msg(r));
  if (gh->is_canceling())                      gh->canceled(msg);
  else if (r.error_code == result_code::kSuccessful) gh->succeed(msg);
  else                                         gh->abort(msg);
}

// v1 free-running JointState stream (from the supervisor pump thread, ~100 Hz).
void Ros2Backend::publish_state(const ArmState& s) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
  msg.position.resize(kinova::kNumJoints);
  msg.velocity.resize(kinova::kNumJoints);
  msg.effort.resize(kinova::kNumJoints);
  for (int i = 0; i < kinova::kNumJoints; ++i) {
    msg.position[i] = s.q[i];
    msg.velocity[i] = s.qd[i];
    msg.effort[i] = s.tau[i];
  }

  if (gripper_ != nullptr) {
    // ONE joint: the 2F-85 is underactuated -- one revolute DOF and five <mimic> joints
    // at +/-1, which robot_state_publisher derives itself (verified 2026-09-03).
    // Publishing the dependents too would duplicate what the model already states.
    //
    // Published REGARDLESS of `present`: omitting it would drop the gripper out of TF
    // entirely -- its links would just vanish from the model rather than render at a
    // frozen or default pose (verified 2026-09-03: RSP publishes per-segment, not
    // all-or-nothing for the whole robot; the arm's TF is unaffected either way).
    // /gripper_state carries the truth about presence instead.
    //
    // This is a second snap_ load inside core, so the value can be up to one 1 kHz
    // feedback frame newer than `s`. Irrelevant for TF; noted because the conformance
    // suite's same-pump-tick invariant covers joint_states and ee_state, not this column.
    const auto g = gripper_->on_query_gripper();
    msg.name.push_back("robotiq_85_left_knuckle_joint");
    msg.position.push_back(gripper_to_knuckle_rad(g.position));
    // NaN, never 0: sensor_msgs' convention for "no measurement". Zero would be
    // indistinguishable from "not moving" / "no load". Core has no gripper velocity at
    // all (it removed the field), and its effort is a 0..1 current fraction, not N*m.
    msg.velocity.push_back(std::numeric_limits<double>::quiet_NaN());
    msg.effort.push_back(std::numeric_limits<double>::quiet_NaN());
  }

  state_pub_->publish(msg);

  kinova_arm_interfaces::msg::EeState ee;
  ee.header.stamp = msg.header.stamp;          // same tick as the joint state above
  ee.pose.position.x = s.ee_pose.p.x();
  ee.pose.position.y = s.ee_pose.p.y();
  ee.pose.position.z = s.ee_pose.p.z();
  ee.pose.orientation.w = s.ee_pose.R.w();
  ee.pose.orientation.x = s.ee_pose.R.x();
  ee.pose.orientation.y = s.ee_pose.R.y();
  ee.pose.orientation.z = s.ee_pose.R.z();
  ee.twist.linear.x  = s.ee_twist[0];           // core packs [linear; angular]
  ee.twist.linear.y  = s.ee_twist[1];
  ee.twist.linear.z  = s.ee_twist[2];
  ee.twist.angular.x = s.ee_twist[3];
  ee.twist.angular.y = s.ee_twist[4];
  ee.twist.angular.z = s.ee_twist[5];
  ee_pub_->publish(ee);

  fault_.store(s.fault);
  arm_stamp_s_.store(s.stamp_s);
  ever_published_.store(true);
}

void Ros2Backend::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const bool ever = ever_published_.load();
  const bool fault = fault_.load();
  // STALE rather than OK when nothing has arrived: an arm we have never heard from
  // is not a healthy arm, and reporting OK for it is the failure mode REP 107's
  // levels exist to avoid.
  if (!ever)      stat.summary(DiagnosticStatus::STALE, "no feedback from the arm yet");
  else if (fault) stat.summary(DiagnosticStatus::ERROR, "arm reports a fault");
  else            stat.summary(DiagnosticStatus::OK, "OK");
  stat.add("fault", fault);
  stat.add("driver_uptime_s", arm_stamp_s_.load());
  stat.add("feedback_received", ever);
}
}  // namespace kinova_arm_ros2
