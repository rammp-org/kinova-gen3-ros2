#pragma once
#include <future>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2::test {

// Minimal fake /rammp_curobo/plan_to_pose server. succeed=true returns a canned
// n-point joint_1..7 trajectory; succeed=false aborts with a message. reject=true
// rejects the goal outright (goal_response_callback sees a null handle).
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
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanToPose>;

  FakeCuroboServer(rclcpp::Node::SharedPtr node, bool succeed, int n_points = 3,
                    bool reject = false, std::shared_future<void> gate = {},
                    std::shared_ptr<std::promise<void>> started = nullptr,
                    bool reject_cancel = false, bool bad_width = false)
      : node_(node), succeed_(succeed), n_points_(n_points), reject_(reject),
        gate_(std::move(gate)), started_(std::move(started)),
        reject_cancel_(reject_cancel), bad_width_(bad_width) {
    server_ = rclcpp_action::create_server<PlanToPose>(
        node_, "/rammp_curobo/plan_to_pose",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanToPose::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                          : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandle>) {
          return reject_cancel_ ? rclcpp_action::CancelResponse::REJECT
                                 : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> gh) { execute(gh); });
  }

 private:
  void execute(std::shared_ptr<GoalHandle> gh) {
    if (started_) started_->set_value();
    if (gate_.valid()) gate_.wait();
    auto result = std::make_shared<PlanToPose::Result>();
    if (!succeed_) {
      result->success = false;
      result->message = "fake planner: no solution";
      gh->abort(result);
      return;
    }
    result->success = true;
    result->message = "fake plan ok";
    result->trajectory.joint_names =
        {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
    for (int k = 0; k < n_points_; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign((bad_width_ && k == 0) ? 4 : 7, 0.01 * (k + 1));
      const double t = 0.02 * (k + 1);
      p.time_from_start.sec = static_cast<int32_t>(t);
      p.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
      result->trajectory.points.push_back(p);
    }
    gh->succeed(result);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<PlanToPose>::SharedPtr server_;
  bool succeed_;
  int n_points_;
  bool reject_;
  std::shared_future<void> gate_;
  std::shared_ptr<std::promise<void>> started_;
  bool reject_cancel_;
  bool bad_width_;
};
}  // namespace kinova_arm_ros2::test
