#pragma once
#include <optional>
#include <string>
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_ros2/planned_move_server.h"
namespace kinova_arm_ros2 {

// Hosts GoToEEPose: validate -> cuRobo plan_to_pose -> feed the planned
// trajectory into the shared CommandSink seam (same path as
// ExecuteJointTrajectory) -> settle. The lifecycle lives in PlannedMoveServer;
// only the frame check and the planner call are specific to this action.
class GoToEEPoseServer : public PlannedMoveServer<kinova_arm_interfaces::action::GoToEEPose> {
 public:
  using Action = kinova_arm_interfaces::action::GoToEEPose;

  GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                   CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_ee_pose", router, planner, cb_group) {}

 protected:
  std::optional<std::string> validate(const Action::Goal& goal) override {
    if (goal.target.header.frame_id != "base_link")
      return "GoToEEPose: frame_id '" + goal.target.header.frame_id + "' != base_link";
    return std::nullopt;
  }

  void start_plan(const Action::Goal& goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan(goal.target.pose, this->start_config(), std::move(on_fb),
                  std::move(on_done));
  }
};

}  // namespace kinova_arm_ros2
