#include "kinova_arm_ros2/curobo_plan_client.h"
#include <chrono>
#include <mutex>
namespace kinova_arm_ros2 {

CuroboPlanClient::CuroboPlanClient(rclcpp::Node::SharedPtr node,
                                   rclcpp::CallbackGroup::SharedPtr cb_group,
                                   std::string action_name)
    : node_(node) {
  client_ = rclcpp_action::create_client<PlanToPose>(node_, action_name, cb_group);
}

void CuroboPlanClient::plan(const geometry_msgs::msg::Pose& target,
                            FeedbackCb on_fb, DoneCb on_done) {
  // Guarantee on_done fires exactly once across the goal-response / result paths.
  auto once = std::make_shared<std::once_flag>();
  auto fire = [once, on_done](Outcome o) {
    std::call_once(*once, [&] { on_done(std::move(o)); });
  };

  if (!client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    fire({false, "cuRobo action server unavailable", {}});
    return;
  }

  PlanToPose::Goal goal;
  goal.target = target;   // start_joints left empty -> cuRobo reads our /joint_states

  rclcpp_action::Client<PlanToPose>::SendGoalOptions opts;
  opts.goal_response_callback = [this, fire](std::shared_ptr<GoalHandle> gh) {
    if (!gh) { fire({false, "cuRobo rejected plan goal", {}}); return; }
    std::lock_guard<std::mutex> l(m_);
    active_ = gh;
  };
  opts.feedback_callback = [on_fb](std::shared_ptr<GoalHandle>,
                                   const std::shared_ptr<const PlanToPose::Feedback> fb) {
    if (on_fb) on_fb(fb->state);
  };
  opts.result_callback = [this, fire](const GoalHandle::WrappedResult& wr) {
    { std::lock_guard<std::mutex> l(m_); active_.reset(); }
    Outcome o;
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result && wr.result->success) {
      o.ok = true;
      o.message = wr.result->message;
      o.trajectory = wr.result->trajectory;
    } else {
      o.ok = false;
      o.message = (wr.result && !wr.result->message.empty()) ? wr.result->message
                                                             : "cuRobo plan failed";
    }
    fire(std::move(o));
  };
  client_->async_send_goal(goal, opts);
}

void CuroboPlanClient::cancel() {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_); gh = active_; }
  if (gh) client_->async_cancel_goal(gh);
}
}  // namespace kinova_arm_ros2
