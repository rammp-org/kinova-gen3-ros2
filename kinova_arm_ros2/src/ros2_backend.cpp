// kinova_arm_ros2/src/ros2_backend.cpp
#include "kinova_arm_ros2/ros2_backend.h"
#include <functional>
#include "kinova_lowlevel/joint_types.h"
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
  state_pub_->publish(msg);
}
}  // namespace kinova_arm_ros2
