#pragma once
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_pose.hpp"
#include "rammp_curobo_interfaces/action/blend.hpp"
#include "rammp_curobo_interfaces/action/plan_to_joints.hpp"
#include "rammp_curobo_interfaces/action/plan_to_pose.hpp"
namespace kinova_gen3_ros2 {

// Async client to the external cuRobo planner (/rammp_curobo/plan_to_pose,
// /rammp_curobo/plan_to_joints and /rammp_curobo/blend). The ONLY unit that
// knows cuRobo exists. plan()/plan_to_joints()/blend() dispatch and return; the
// result arrives on on_done from the rclcpp executor (client's reentrant
// group). on_done is invoked EXACTLY ONCE (success, failure, rejection, or
// unavailable).
//
// THREADING: on_done normally runs on the rclcpp executor, but some failures
// invoke it SYNCHRONOUSLY on the caller's thread before
// plan()/plan_to_joints()/ blend() returns -- e.g. an unknown hold mode, or an
// unavailable action server. A caller must not hold a lock across these calls
// that on_done also takes.
class CuroboPlanClient {
public:
  using PlanToPose = rammp_curobo_interfaces::action::PlanToPose;
  using PlanToJoints = rammp_curobo_interfaces::action::PlanToJoints;
  using Blend = rammp_curobo_interfaces::action::Blend;
  struct Outcome {
    bool ok = false;
    std::string message;
    trajectory_msgs::msg::JointTrajectory trajectory;
    // How far the planned goal landed from the requested joints. cuRobo plans
    // joint goals natively where it can (~0) but falls back to an FK pose,
    // which can converge somewhere else. Always 0 for pose plans, which have no
    // joint-space request to miss.
    double goal_mismatch_rad = 0.0;
  };
  using FeedbackCb = std::function<void(const std::string &state)>;
  using DoneCb = std::function<void(Outcome)>;

  CuroboPlanClient(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cb_group,
      std::string action_name = "/rammp_curobo/plan_to_pose",
      std::string joints_action_name = "/rammp_curobo/plan_to_joints",
      std::string blend_action_name = "/rammp_curobo/blend");
  // start_joints is the configuration the plan must START from -- the arm's
  // measured q, supplied by the caller. It is NOT optional: cuRobo treats an
  // empty vector as "subscribe to /joint_states and work it out yourself",
  // which couples the planner to the robot and lets it plan from a state up to
  // 2 s stale while we execute from now.
  void plan(const geometry_msgs::msg::Pose &target,
            const std::vector<double> &start_joints, FeedbackCb on_fb,
            DoneCb on_done);
  // As above, constrained by the arm's own orientation hold (one of
  // GoToEEPose::Goal::HOLD_*). It is translated into the planner's equivalent
  // here and nowhere else, so replacing cuRobo never changes what an arm
  // client sends. HOLD_NONE means "no constraint".
  void plan(const geometry_msgs::msg::Pose &target,
            const std::vector<double> &start_joints, uint8_t hold,
            FeedbackCb on_fb, DoneCb on_done);
  void plan_to_joints(const std::vector<double> &target_joints,
                      const std::vector<double> &start_joints, FeedbackCb on_fb,
                      DoneCb on_done);
  // Blends legs this planner returned into ONE trajectory that passes each
  // junction still moving where its blend_radius allows. Each leg must start
  // where the one before it ends (its last point) and run rest to rest.
  // blend_radius: one per junction (m), legs.size() - 1 of them. speed_scale:
  // one per leg, or empty for all 1. hold: one per leg, the arm's
  // GoToEEPose::Goal::HOLD_* the leg was planned with, or empty for none;
  // translated here as plan() translates it. hold_orientation: one per leg,
  // the orientation a held leg's hold keeps (its goal's, as sent to plan()),
  // read only for held legs; or empty. The trajectory comes back at the legs'
  // speed scales, so it is executed at speed scale 1.
  //
  // ONE PLAN AT A TIME, as for every call here: the planner refuses a second
  // goal while one is planning ("planner busy"), and cancel() reaches only the
  // goal started last. A caller chaining several calls (plan, plan, blend)
  // shares the planner with every other GoTo goal in between.
  void
  blend(const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
        const std::vector<double> &blend_radius,
        const std::vector<double> &speed_scale,
        const std::vector<uint8_t> &hold,
        const std::vector<geometry_msgs::msg::Quaternion> &hold_orientation,
        FeedbackCb on_fb, DoneCb on_done);
  // Cancels whichever plan is in flight: pose, joint-space or blend.
  void cancel();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanToPose>::SharedPtr client_;
  rclcpp_action::Client<PlanToJoints>::SharedPtr client_joints_;
  rclcpp_action::Client<Blend>::SharedPtr client_blend_;
  std::mutex m_;
  // Type-erased cancel for the in-flight goal, so one cancel() serves all
  // three action types. Null when nothing is in flight.
  std::function<void()> active_cancel_;
};
} // namespace kinova_gen3_ros2
