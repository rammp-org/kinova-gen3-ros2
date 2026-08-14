#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2::test {

// Minimal fake /rammp_curobo/plan_to_pose server. succeed=true returns a canned
// n-point joint_1..7 trajectory; succeed=false aborts with a message. reject=true
// rejects the goal outright (goal_response_callback sees a null handle).
class FakeCuroboServer {
 public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanToPose>;

  FakeCuroboServer(rclcpp::Node::SharedPtr node, bool succeed, int n_points = 3,
                    bool reject = false)
      : node_(node), succeed_(succeed), n_points_(n_points), reject_(reject) {
    server_ = rclcpp_action::create_server<PlanToPose>(
        node_, "/rammp_curobo/plan_to_pose",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanToPose::Goal>) {
          return reject_ ? rclcpp_action::GoalResponse::REJECT
                          : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<GoalHandle> gh) { execute(gh); });
  }

 private:
  void execute(std::shared_ptr<GoalHandle> gh) {
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
      p.positions.assign(7, 0.01 * (k + 1));
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
};
}  // namespace kinova_arm_ros2::test
