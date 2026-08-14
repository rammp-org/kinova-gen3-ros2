#include "kinova_arm_ros2/goto_ee_pose_server.h"
#include <functional>
#include "kinova_arm_ros2/message_mapping.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_arm_ros2 {
using namespace kinova::interface;
using std::placeholders::_1;
using std::placeholders::_2;

GoToEEPoseServer::GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                                   CuroboPlanClient& planner,
                                   rclcpp::CallbackGroup::SharedPtr cb_group)
    : node_(node), router_(router), planner_(planner) {
  server_ = rclcpp_action::create_server<Action>(
      node_, "go_to_ee_pose",
      std::bind(&GoToEEPoseServer::handle_goal, this, _1, _2),
      std::bind(&GoToEEPoseServer::handle_cancel, this, _1),
      std::bind(&GoToEEPoseServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(), cb_group);
}

rclcpp_action::GoalResponse GoToEEPoseServer::handle_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
  if (!sink_) return rclcpp_action::GoalResponse::REJECT;
  if (goal->target.header.frame_id != "base_link") {   // fail loud
    RCLCPP_WARN(node_->get_logger(),
                "rejecting GoToEEPose: frame_id '%s' != base_link",
                goal->target.header.frame_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToEEPoseServer::handle_cancel(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  bool executing = false;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it != goals_.end()) executing = it->second.executing; }
  if (executing) {
    if (sink_) sink_->on_trajectory_cancel(id);   // Supervisor -> kPreempted -> settle()
  } else {
    planner_.cancel();                            // cancel in-flight plan -> on_plan_done
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToEEPoseServer::handle_accepted(std::shared_ptr<GoalHandle> gh) {
  const GoalId id = gh->get_goal_id();
  { std::lock_guard<std::mutex> l(m_); goals_[id] = Goal{gh, false}; }
  auto planning = std::make_shared<Action::Feedback>();
  planning->phase = "planning";
  gh->publish_feedback(planning);

  const geometry_msgs::msg::Pose target = gh->get_goal()->target.pose;
  planner_.plan(
      target,
      [this, id](const std::string& state) {
        std::shared_ptr<GoalHandle> gh2;
        { std::lock_guard<std::mutex> l(m_);
          auto it = goals_.find(id);
          if (it == goals_.end()) return;
          gh2 = it->second.gh; }
        auto f = std::make_shared<Action::Feedback>();
        f->phase = "planning";
        f->planner_state = state;
        gh2->publish_feedback(f);
      },
      [this, id](CuroboPlanClient::Outcome o) { on_plan_done(id, std::move(o)); });
}

void GoToEEPoseServer::on_plan_done(GoalId id, CuroboPlanClient::Outcome outcome) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh; }

  if (!outcome.ok) {
    const bool canceled = gh->is_canceling();
    settle_local(gh, canceled ? result_code::kPreempted : result_code::kPlanningFailed,
                 canceled ? "canceled during planning" : outcome.message);
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }

  TrajectoryGoal tg = to_trajectory_goal(outcome.trajectory);   // position mode
  tg.path_tolerance = kinova::JointVec::Constant(kGotoPathTolRad);
  tg.sender_id = gh->get_goal()->sender_id;

  const GoalResponse r = sink_->on_trajectory_goal(tg);
  if (r != GoalResponse::kAccept) {
    settle_local(gh, result_code::kInvalidGoal, "supervisor rejected planned trajectory");
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it != goals_.end()) it->second.executing = true; }
  router_.register_owner(id, *this);
  sink_->on_trajectory_accepted(id, tg);
}

void GoToEEPoseServer::settle_local(std::shared_ptr<GoalHandle> gh, int error_code,
                                    const std::string& msg) {
  TrajectoryResult r;
  r.error_code = error_code;
  r.error_string = msg;
  r.final_error = kinova::JointVec::Zero();
  auto out = std::make_shared<Action::Result>(to_goto_result_msg(r));
  if (gh->is_canceling())                          gh->canceled(out);
  else if (error_code == result_code::kSuccessful) gh->succeed(out);
  else                                             gh->abort(out);
}

// --- ActionServerPort (execution phase, sampler thread via router) ---
void GoToEEPoseServer::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh; }
  auto msg = std::make_shared<Action::Feedback>(to_goto_feedback_msg(fb));
  gh->publish_feedback(msg);
}

void GoToEEPoseServer::settle(const GoalId& id, const TrajectoryResult& r) {
  std::shared_ptr<GoalHandle> gh;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it == goals_.end()) return;
    gh = it->second.gh;
    goals_.erase(it); }
  auto msg = std::make_shared<Action::Result>(to_goto_result_msg(r));
  if (gh->is_canceling())                            gh->canceled(msg);
  else if (r.error_code == result_code::kSuccessful) gh->succeed(msg);
  else                                               gh->abort(msg);
}
}  // namespace kinova_arm_ros2
