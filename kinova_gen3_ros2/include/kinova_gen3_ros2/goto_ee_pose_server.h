#pragma once
#include <optional>
#include <string>
#include "rammp_arm_interfaces/msg/approach_offset.hpp"
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
    // An approach frees its own axis (the planner travels along it), so locking
    // that same axis asks for two opposite things. A zero distance means no
    // approach is active, and must not trip this.
    if (goal.approach_offset.distance != 0.0) {
      using Approach = rammp_arm_interfaces::msg::ApproachOffset;
      const bool locked =
          (goal.approach_offset.axis == Approach::AXIS_X &&
           goal.axis_lock.lock_x) ||
          (goal.approach_offset.axis == Approach::AXIS_Y &&
           goal.axis_lock.lock_y) ||
          (goal.approach_offset.axis == Approach::AXIS_Z &&
           goal.axis_lock.lock_z);
      if (locked)
        return std::string("GoToEEPose: approach_offset.axis (") +
               "XYZ"[goal.approach_offset.axis] +
               ") travels along an axis that axis_lock.lock_" +
               "xyz"[goal.approach_offset.axis] +
               " locks; an approach frees its own axis, so drop the lock "
               "or the approach";
    }
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan(goal.target.pose, this->start_config(), goal.axis_lock,
                  goal.approach_offset, std::move(on_fb), std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
