#pragma once
#include <optional>
#include <string>
#include "rammp_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_gen3_ros2/planned_move_server.h"
namespace kinova_gen3_ros2 {

// Hosts GoToEEPose: validate -> cuRobo plan_to_pose -> feed the planned
// trajectory into the shared CommandSink seam (same path as
// ExecuteJointTrajectory) -> settle. The lifecycle lives in PlannedMoveServer;
// only the frame check and the planner call are specific to this action.
class GoToEEPoseServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToEEPose> {
public:
  using Action = rammp_arm_interfaces::action::GoToEEPose;

  GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                   CuroboPlanClient &planner,
                   rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_ee_pose", router, planner,
                                  cb_group) {}

protected:
  std::optional<std::string> validate(const Action::Goal &goal) override {
    if (goal.target.header.frame_id != "base_link")
      return "GoToEEPose: frame_id '" + goal.target.header.frame_id +
             "' != base_link";
    if (auto why = speed_scale_rejection(goal.speed_scale))
      return "GoToEEPose: " + *why;
    // Reject an unknown mode HERE rather than letting it reach the planner.
    // Defaulting it to HOLD_NONE would run an unconstrained move for a caller
    // who asked for a held one, which is worse than refusing because it looks
    // like success. (CuroboPlanClient refuses it too -- this is the earlier of
    // the two gates, so the client gets a goal rejection instead of a failed
    // result.)
    switch (goal.orientation_hold) {
    case Action::Goal::HOLD_NONE:
    case Action::Goal::HOLD_LEVEL:
    case Action::Goal::HOLD_FIXED:
      break;
    default:
      return "GoToEEPose: unknown orientation_hold " +
             std::to_string(goal.orientation_hold) +
             " (expected HOLD_NONE=0, HOLD_LEVEL=1 or HOLD_FIXED=2)";
    }
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan(goal.target.pose, this->start_config(), goal.orientation_hold,
                  std::move(on_fb), std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
