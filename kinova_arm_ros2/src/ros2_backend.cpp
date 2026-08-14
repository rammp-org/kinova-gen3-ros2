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
  const CancelResponse r = sink_->on_trajectory_cancel(gh->get_goal_id());   // GoalUUID -> GoalId
  return (r == CancelResponse::kAccept) ? rclcpp_action::CancelResponse::ACCEPT
                                        : rclcpp_action::CancelResponse::REJECT;
}

void Ros2Backend::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  { std::lock_guard<std::mutex> l(m_); handles_[id] = gh; }
  sink_->on_trajectory_accepted(id, to_trajectory_goal(*gh->get_goal()));
}

void Ros2Backend::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second; }
  auto msg = std::make_shared<Action::Feedback>(to_feedback_msg(id, fb));
  gh->publish_feedback(msg);
}

void Ros2Backend::settle(const GoalId& id, const TrajectoryResult& r) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); auto it = handles_.find(id); if (it == handles_.end()) return; gh = it->second; handles_.erase(it); }
  auto msg = std::make_shared<Action::Result>(to_result_msg(r));
  if (gh->is_canceling())                      gh->canceled(msg);
  else if (r.error_code == result_code::kSuccessful) gh->succeed(msg);
  else                                         gh->abort(msg);
}
}  // namespace kinova_arm_ros2
