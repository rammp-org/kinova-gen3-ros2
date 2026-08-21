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
    if (it != goals_.end()) { executing = it->second.executing;
                              it->second.cancel_requested = true; } }
  if (executing) {
    if (sink_) sink_->on_trajectory_cancel(id);   // Supervisor -> kPreempted -> settle()
  } else {
    // Still planning, OR mid-handover. planner_.cancel() covers the first; the
    // second is covered by on_plan_done re-reading cancel_requested AFTER
    // on_trajectory_accepted, because a cancel delivered before the supervisor
    // owns the goal drains against nothing on its single FIFO inbox.
    planner_.cancel();
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

  if (gh->is_canceling()) {   // canceled during planning; plan raced ahead and succeeded
    settle_local(gh, result_code::kPreempted, "canceled during planning");
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }

  // Fail loud on a malformed planner output rather than let to_trajectory_goal's
  // memory-safety zero-fill silently mis-map a short/empty point onto the arm.
  if (outcome.trajectory.points.empty()) {
    settle_local(gh, result_code::kPlanningFailed, "planner returned an empty trajectory");
    { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
    return;
  }
  for (const auto& p : outcome.trajectory.points) {
    if (p.positions.size() != static_cast<size_t>(kinova::kNumJoints)) {
      settle_local(gh, result_code::kPlanningFailed,
                   "planner returned malformed trajectory: point has " +
                       std::to_string(p.positions.size()) + " positions, expected " +
                       std::to_string(kinova::kNumJoints));
      { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }
      return;
    }
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

  // The supervisor now owns the goal. If a cancel landed any time before this
  // point it was ACCEPTed but could not have stopped anything -- either the
  // plan had already finished (planner_.cancel() was a no-op) or the cancel
  // reached the supervisor's FIFO ahead of the goal and drained against no
  // active trajectory. Re-issue it now, when it can actually take effect.
  bool canceled = false;
  { std::lock_guard<std::mutex> l(m_);
    auto it = goals_.find(id);
    if (it != goals_.end()) canceled = it->second.cancel_requested; }
  if (canceled) sink_->on_trajectory_cancel(id);
}

namespace {
// rclcpp_action throws std::runtime_error if the goal has already left the state
// this transition expects -- and is_canceling() can flip between the check and
// the call, because handle_cancel ACCEPTs from an executor thread. settle() runs
// on Supervisor::sampler_loop, which has no handler, so an escaping exception
// terminates the process with the arm powered and mid-trajectory. Losing the
// terminal transition is bad; losing the process is worse.
template <class Handle, class Msg>
void terminate_goal(rclcpp::Logger log, const Handle& gh, const Msg& msg,
                    int error_code) {
  try {
    if (gh->is_canceling())                          gh->canceled(msg);
    else if (error_code == kinova::interface::result_code::kSuccessful)
                                                     gh->succeed(msg);
    else                                             gh->abort(msg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(log, "goal terminal transition raced a cancel and was dropped: %s",
                e.what());
  }
}
}  // namespace

void GoToEEPoseServer::settle_local(std::shared_ptr<GoalHandle> gh, int error_code,
                                    const std::string& msg) {
  TrajectoryResult r;
  r.error_code = error_code;
  r.error_string = msg;
  r.final_error = kinova::JointVec::Zero();
  auto out = std::make_shared<Action::Result>(to_goto_result_msg(r));
  terminate_goal(node_->get_logger(), gh, out, error_code);
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
  terminate_goal(node_->get_logger(), gh, msg, r.error_code);
}
}  // namespace kinova_arm_ros2
