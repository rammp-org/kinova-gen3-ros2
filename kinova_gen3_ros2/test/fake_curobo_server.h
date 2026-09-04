#pragma once
#include <future>
#include <memory>
#include <mutex>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_curobo_interfaces/action/plan_to_joints.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_gen3_ros2::test {

// Minimal fake cuRobo planner. Hosts BOTH /rammp_curobo/plan_to_pose and
// /rammp_curobo/plan_to_joints off the same configuration, so a test picks a
// planning tier purely by which one it calls. succeed=true returns a canned
// n-point joint_1..7 trajectory; succeed=false aborts with a message.
// reject=true rejects the goal outright (goal_response_callback sees a null
// handle).
//
// Optional, additive knobs for deterministically testing races against a
// caller that cancels while planning is in flight:
//   - gate: if valid, execute() blocks on it before producing a result, so a
//     test can pin down exactly when planning "finishes".
//   - started: if non-null, set once execute() begins (before waiting on
//     gate) so a test can deterministically know planning has started.
//   - reject_cancel: if true, this server's OWN cancel handler REJECTs
//     incoming cancel requests -- models a planner that can't be interrupted
//     once committed, so execute() can still legitimately succeed() even
//     after a cancel was requested against its goal (without racing an
//     invalid CANCELING->SUCCEEDED transition in rclcpp_action).
//   - bad_width: if true, the first point of an otherwise-successful plan
//     carries only 4 positions instead of 7 -- models a malformed/buggy
//     planner response for testing the caller's fail-loud width guard.
class FakeCuroboServer {
public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using PlanToJoints = rammp_curobo_interfaces::action::PlanToJoints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanToPose>;
  using JointsGoalHandle = rclcpp_action::ServerGoalHandle<PlanToJoints>;

  FakeCuroboServer(rclcpp::Node::SharedPtr node, bool succeed, int n_points = 3,
                   bool reject = false, std::shared_future<void> gate = {},
                   std::shared_ptr<std::promise<void>> started = nullptr,
                   bool reject_cancel = false, bool bad_width = false)
      : node_(node), succeed_(succeed), n_points_(n_points), reject_(reject),
        gate_(std::move(gate)), started_(std::move(started)),
        reject_cancel_(reject_cancel), bad_width_(bad_width) {
    server_ = rclcpp_action::create_server<PlanToPose>(
        node_, "/rammp_curobo/plan_to_pose",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const PlanToPose::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandle>) {
          return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT
                                : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> gh) { execute<PlanToPose>(gh); });
    joints_server_ = rclcpp_action::create_server<PlanToJoints>(
        node_, "/rammp_curobo/plan_to_joints",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const PlanToJoints::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<JointsGoalHandle>) {
          return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT
                                : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<JointsGoalHandle> gh) {
          execute<PlanToJoints>(gh);
        });
  }

  // What the caller actually asked us to plan FROM. Empty means the caller
  // left start_joints unset, which tells the real cuRobo node to go read
  // /joint_states itself -- the coupling this records the absence of.
  std::vector<double> last_start_joints() const {
    std::lock_guard<std::mutex> l(seen_m_);
    return last_start_joints_;
  }

private:
  // Shared by both tiers; only the Result type differs. PlanToJoints::Result
  // additionally carries goal_mismatch_rad, which stays at its 0.0 default -
  // the canned plan is treated as reaching the requested joints exactly.
  // `started_` is a one-shot promise, so a single test must drive only one
  // tier.
  template <typename ActionT>
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh) {
    {
      std::lock_guard<std::mutex> l(seen_m_);
      last_start_joints_ = gh->get_goal()->start_joints;
    }
    if (started_)
      started_->set_value();
    if (gate_.valid())
      gate_.wait();
    auto result = std::make_shared<typename ActionT::Result>();
    if (!succeed_) {
      result->success = false;
      result->message = "fake planner: no solution";
      gh->abort(result);
      return;
    }
    result->success = true;
    result->message = "fake plan ok";
    result->trajectory.joint_names = {"joint_1", "joint_2", "joint_3",
                                      "joint_4", "joint_5", "joint_6",
                                      "joint_7"};
    for (int k = 0; k < n_points_; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign((bad_width_ && k == 0) ? 4 : 7, 0.01 * (k + 1));
      const double t = 0.02 * (k + 1);
      p.time_from_start.sec = static_cast<int32_t>(t);
      p.time_from_start.nanosec =
          static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
      result->trajectory.points.push_back(p);
    }
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<PlanToPose>::SharedPtr server_;
  rclcpp_action::Server<PlanToJoints>::SharedPtr joints_server_;
  bool succeed_;
  int n_points_;
  bool reject_;
  std::shared_future<void> gate_;
  std::shared_ptr<std::promise<void>> started_;
  bool reject_cancel_;
  bool bad_width_;
  mutable std::mutex seen_m_;
  std::vector<double> last_start_joints_;
};
} // namespace kinova_gen3_ros2::test
