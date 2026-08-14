#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rammp_curobo_interfaces/action/plan_to_joints.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_arm_ros2 {

// Async client to the external cuRobo planner (/rammp_curobo/plan_to_pose and
// /rammp_curobo/plan_to_joints). The ONLY unit that knows cuRobo exists.
// plan()/plan_to_joints() dispatch and return; the result arrives on on_done
// from the rclcpp executor (client's reentrant group). on_done is invoked
// EXACTLY ONCE (success, failure, rejection, or unavailable).
class CuroboPlanClient {
 public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using PlanToJoints = rammp_curobo_interfaces::action::PlanToJoints;
  struct Outcome {
    bool ok = false;
    std::string message;
    trajectory_msgs::msg::JointTrajectory trajectory;
    // How far the planned goal landed from the requested joints. cuRobo plans
    // joint goals natively where it can (~0) but falls back to an FK pose, which
    // can converge somewhere else. Always 0 for pose plans, which have no
    // joint-space request to miss.
    double goal_mismatch_rad = 0.0;
  };
  using FeedbackCb = std::function<void(const std::string& state)>;
  using DoneCb = std::function<void(Outcome)>;

  CuroboPlanClient(rclcpp::Node::SharedPtr node,
                   rclcpp::CallbackGroup::SharedPtr cb_group,
                   std::string action_name = "/rammp_curobo/plan_to_pose",
                   std::string joints_action_name = "/rammp_curobo/plan_to_joints");
  void plan(const geometry_msgs::msg::Pose& target, FeedbackCb on_fb, DoneCb on_done);
  void plan_to_joints(const std::vector<double>& target_joints, FeedbackCb on_fb, DoneCb on_done);
  // Cancels whichever plan is in flight, pose or joint-space.
  void cancel();

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanToPose>::SharedPtr client_;
  rclcpp_action::Client<PlanToJoints>::SharedPtr client_joints_;
  std::mutex m_;
  // Type-erased cancel for the in-flight goal, so one cancel() serves both
  // action types. Null when nothing is in flight.
  std::function<void()> active_cancel_;
};
}  // namespace kinova_arm_ros2
