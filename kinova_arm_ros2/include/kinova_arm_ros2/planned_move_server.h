#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/joint_point.h"
#include "kinova_arm_ros2/message_mapping.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_arm_ros2 {

// Shared plan -> execute -> settle lifecycle for the high-level "go to a goal"
// actions (GoToEEPose, GoToJointConfig, GoToPreset). A concrete server supplies
// only validate() and start_plan(); everything concurrency-critical lives here,
// once.
//
// The invariant this class exists to protect: every goal terminals its
// ServerGoalHandle EXACTLY once - never zero (the client hangs) and never twice
// (rclcpp throws). goals_ is guarded by m_, and no downstream ROS or sink call
// is ever made while holding m_. Each action's Result/Feedback fields are
// identical, so the messages are built inline here rather than per action.
template <class ActionT>
class PlannedMoveServer : public kinova::interface::ActionServerPort {
 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  PlannedMoveServer(rclcpp::Node::SharedPtr node, const std::string& action_name,
                    GoalRouter& router, CuroboPlanClient& planner,
                    rclcpp::CallbackGroup::SharedPtr cb_group)
      : planner_(planner), node_(node), router_(router) {
    using std::placeholders::_1;
    using std::placeholders::_2;
    server_ = rclcpp_action::create_server<ActionT>(
        node_, action_name,
        std::bind(&PlannedMoveServer::handle_goal, this, _1, _2),
        std::bind(&PlannedMoveServer::handle_cancel, this, _1),
        std::bind(&PlannedMoveServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
  }
  virtual ~PlannedMoveServer() = default;

  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

  // --- ActionServerPort (execution phase, supervisor sampler thread via router) ---
  void publish_feedback(const kinova::interface::GoalId& id,
                        const kinova::interface::TrajectoryFeedback& fb) override {
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it == goals_.end()) return;
      gh = it->second.gh; }
    auto msg = std::make_shared<typename ActionT::Feedback>();
    msg->phase = "executing";
    msg->fraction_complete = static_cast<float>(fb.fraction_complete);
    msg->actual = vec_to_point(fb.actual);
    gh->publish_feedback(msg);
  }

  void settle(const kinova::interface::GoalId& id,
              const kinova::interface::TrajectoryResult& r) override {
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it == goals_.end()) return;      // already settled -> never terminal twice
      gh = it->second.gh;
      goals_.erase(it); }
    terminal(gh, r.error_code, r.error_string, r.final_error);
  }

 protected:
  // Everything action-specific. validate() returns a reason to reject, or
  // The arm's measured configuration, for start_plan() to state in the plan
  // request. This node owns that state -- it is the same source /joint_states
  // is published from -- so the planner never has to source it itself.
  std::vector<double> start_config() const {
    // handle_goal has already refused the goal unless sink_ exists and has a
    // measured state, so there is no empty-vector fallback here -- returning one
    // would silently restore the /joint_states coupling this replaced.
    const kinova::interface::ArmState s = sink_->on_query_state();
    std::vector<double> q(kinova::kNumJoints, 0.0);
    for (int i = 0; i < kinova::kNumJoints; ++i) q[i] = s.q[i];
    return q;
  }

  // nullopt to accept; start_plan() dispatches the appropriate cuRobo plan.
  virtual std::optional<std::string> validate(const typename ActionT::Goal& goal) = 0;
  virtual void start_plan(const typename ActionT::Goal& goal,
                          CuroboPlanClient::FeedbackCb on_fb,
                          CuroboPlanClient::DoneCb on_done) = 0;

  CuroboPlanClient& planner_;      // subclasses call plan() / plan_to_joints()
  rclcpp::Node::SharedPtr node_;

 private:
  using GoalId = kinova::interface::GoalId;
  static constexpr double kGotoPathTolRad = 0.35;  // generous; full-speed tracking lag

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!sink_) return rclcpp_action::GoalResponse::REJECT;
    // Refuse before planning if the arm's configuration is not yet known.
    // Supervisor::pump_loop stores a snapshot only after a SUCCESSFUL feedback
    // read, and a default-constructed ArmState is {q = Zero, stamp_s = 0}. Sending
    // that q as the plan's start state would be indistinguishable from a real
    // measurement, and cuRobo would plan from the fully-extended zero pose.
    if (sink_->on_query_state().stamp_s <= 0.0) {
      RCLCPP_WARN(node_->get_logger(),
                  "rejecting goal: no joint state measured yet -- the arm's "
                  "configuration is unknown, so there is nothing to plan from");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (auto why = validate(*goal)) {   // fail loud
      RCLCPP_WARN(node_->get_logger(), "rejecting goal: %s", why->c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> gh) {
    const GoalId id = gh->get_goal_id();
    bool executing = false;
    kinova::interface::Token token{};
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it != goals_.end()) { executing = it->second.executing;
                                token = it->second.token;
                                it->second.cancel_requested = true; } }
    if (executing) {
      // Replay the goal's own token: a ROS cancel carries no payload, and a zero
      // token is refused by the Arbiter under kEnforced.
      if (sink_) sink_->on_trajectory_cancel({id, token});   // Supervisor -> kPreempted -> settle()
    } else {
      // Still planning, OR mid-handover. planner_.cancel() covers the first; the
      // second is covered by start_execution() re-reading cancel_requested AFTER
      // on_trajectory_accepted, because a cancel delivered before the supervisor
      // owns the goal drains against nothing on its single FIFO inbox.
      planner_.cancel();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandle> gh) {
    const GoalId id = gh->get_goal_id();
    { std::lock_guard<std::mutex> l(m_);
      goals_[id] = Goal{gh, false, false, gh->get_goal()->token}; }
    auto planning = std::make_shared<typename ActionT::Feedback>();
    planning->phase = "planning";
    gh->publish_feedback(planning);

    start_plan(*gh->get_goal(),
        [this, id](const std::string& state) {
          std::shared_ptr<GoalHandle> g;
          { std::lock_guard<std::mutex> l(m_);
            auto it = goals_.find(id);
            if (it == goals_.end()) return;
            g = it->second.gh; }
          auto f = std::make_shared<typename ActionT::Feedback>();
          f->phase = "planning";
          f->planner_state = state;
          g->publish_feedback(f);
        },
        [this, id](CuroboPlanClient::Outcome o) { on_plan_done(id, std::move(o)); });
  }

  void on_plan_done(GoalId id, CuroboPlanClient::Outcome outcome) {
    using kinova::interface::result_code::kInvalidGoal;
    using kinova::interface::result_code::kPlanningFailed;
    using kinova::interface::result_code::kPreempted;
    std::shared_ptr<GoalHandle> gh;
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it == goals_.end()) return;
      gh = it->second.gh; }

    if (!outcome.ok) {
      const bool canceled = gh->is_canceling();
      settle_local(gh, canceled ? kPreempted : kPlanningFailed,
                   canceled ? "canceled during planning" : outcome.message);
      erase(id);
      return;
    }
    if (gh->is_canceling()) {   // canceled during planning; plan raced ahead and succeeded
      settle_local(gh, kPreempted, "canceled during planning");
      erase(id);
      return;
    }

    // Fail loud on malformed planner output rather than let to_trajectory_goal's
    // memory-safety zero-fill silently mis-map a short/empty point onto the arm.
    if (outcome.trajectory.points.empty()) {
      settle_local(gh, kPlanningFailed, "planner returned an empty trajectory");
      erase(id);
      return;
    }
    for (const auto& p : outcome.trajectory.points) {
      if (p.positions.size() != static_cast<size_t>(kinova::kNumJoints)) {
        settle_local(gh, kPlanningFailed,
                     "planner returned malformed trajectory: point has " +
                         std::to_string(p.positions.size()) + " positions, expected " +
                         std::to_string(kinova::kNumJoints));
        erase(id);
        return;
      }
    }

    kinova::interface::TrajectoryGoal tg = to_trajectory_goal(outcome.trajectory);
    tg.path_tolerance = kinova::JointVec::Constant(kGotoPathTolRad);
    tg.sender_id = gh->get_goal()->sender_id;
    tg.token     = gh->get_goal()->token;   // the plan inherits the goal's authority

    if (sink_->on_trajectory_goal(tg) != kinova::interface::GoalResponse::kAccept) {
      settle_local(gh, kInvalidGoal, "supervisor rejected planned trajectory");
      erase(id);
      return;
    }
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it != goals_.end()) it->second.executing = true; }
    router_.register_owner(id, *this);
    sink_->on_trajectory_accepted(id, tg);

    // The supervisor now owns the goal. A cancel accepted before this point was
    // ACCEPTed but could not have stopped anything -- either the plan had
    // already finished (planner_.cancel() was a no-op) or the cancel reached the
    // supervisor's FIFO ahead of the goal and drained against no active
    // trajectory. Re-issue it now, when it can actually take effect.
    bool canceled = false;
    kinova::interface::Token cancel_token{};
    { std::lock_guard<std::mutex> l(m_);
      auto it = goals_.find(id);
      if (it != goals_.end()) { canceled = it->second.cancel_requested;
                                cancel_token = it->second.token; } }
    if (canceled) sink_->on_trajectory_cancel({id, cancel_token});
  }

  void settle_local(std::shared_ptr<GoalHandle> gh, int error_code, const std::string& msg) {
    terminal(gh, error_code, msg, kinova::JointVec::Zero());
  }

  // The single place a goal handle is terminated.
  void terminal(std::shared_ptr<GoalHandle> gh, int error_code, const std::string& msg,
                const kinova::JointVec& final_error) {
    auto out = std::make_shared<typename ActionT::Result>();
    out->error_code = error_code;
    out->error_string = msg;
    out->final_error = vec_to_point(final_error);
    // rclcpp_action throws if the goal has already left the state this
    // transition expects, and is_canceling() can flip between the check and the
    // call because handle_cancel ACCEPTs from an executor thread. settle() runs
    // on Supervisor::sampler_loop, which has no handler, so an escaping
    // exception terminates the node with the arm powered and mid-trajectory.
    // Losing a terminal transition is bad; losing the process is worse.
    try {
      if (gh->is_canceling())                                              gh->canceled(out);
      else if (error_code == kinova::interface::result_code::kSuccessful)  gh->succeed(out);
      else                                                                 gh->abort(out);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
                  "goal terminal transition raced a cancel and was dropped: %s", e.what());
    }
  }

  void erase(const GoalId& id) { std::lock_guard<std::mutex> l(m_); goals_.erase(id); }

  GoalRouter& router_;
  kinova::interface::CommandSink* sink_ = nullptr;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  // cancel_requested latches a cancel that arrived before the supervisor owned
  // the goal, so start_execution() can re-issue it once the handover is done.
  // token: the goal's arbitration capability, kept so cancel can replay it. A ROS
  // action cancel carries no payload, so a cancel with a zero token would be REFUSED
  // by the Arbiter under kEnforced and the motion would run on.
  struct Goal { std::shared_ptr<GoalHandle> gh; bool executing = false;
                bool cancel_requested = false; kinova::interface::Token token{}; };
  std::mutex m_;
  std::map<GoalId, Goal> goals_;
};

}  // namespace kinova_arm_ros2
