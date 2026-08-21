#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2 {

// Async client to the external cuRobo planner (/rammp_curobo/plan_to_pose).
// The ONLY unit that knows cuRobo exists. plan() dispatches and returns; the
// result arrives on on_done from the rclcpp executor (client's reentrant group).
// on_done is invoked EXACTLY ONCE (success, failure, rejection, or unavailable).
class CuroboPlanClient {
 public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  struct Outcome {
    bool ok = false;
    std::string message;
    trajectory_msgs::msg::JointTrajectory trajectory;
  };
  using FeedbackCb = std::function<void(const std::string& state)>;
  using DoneCb = std::function<void(Outcome)>;

  CuroboPlanClient(rclcpp::Node::SharedPtr node,
                   rclcpp::CallbackGroup::SharedPtr cb_group,
                   std::string action_name = "/rammp_curobo/plan_to_pose");
  void plan(const geometry_msgs::msg::Pose& target, FeedbackCb on_fb, DoneCb on_done);
  void cancel();

 private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlanToPose>;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanToPose>::SharedPtr client_;
  std::mutex m_;
  std::shared_ptr<GoalHandle> active_;   // in-flight goal, for cancel()
};
}  // namespace kinova_arm_ros2
