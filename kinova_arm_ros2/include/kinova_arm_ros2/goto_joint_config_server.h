#pragma once
#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include "kinova_arm_interfaces/action/go_to_joint_config.hpp"
#include "kinova_arm_ros2/planned_move_server.h"
namespace kinova_arm_ros2 {

// Hosts GoToJointConfig: move to an explicit 7-joint configuration, planned
// collision-free by cuRobo (plan_to_joints) rather than driven straight there.
// The lifecycle lives in PlannedMoveServer.
class GoToJointConfigServer
    : public PlannedMoveServer<kinova_arm_interfaces::action::GoToJointConfig> {
 public:
  using Action = kinova_arm_interfaces::action::GoToJointConfig;

  GoToJointConfigServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                        CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_joint_config", router, planner, cb_group) {}

 protected:
  std::optional<std::string> validate(const Action::Goal& goal) override {
    // target_joints is float64[7], so the width is type-enforced and cannot be
    // wrong here; finiteness is the guard that actually has work to do. NaN
    // would otherwise reach the planner as a goal it cannot refuse coherently.
    for (size_t i = 0; i < goal.target_joints.size(); ++i)
      if (!std::isfinite(goal.target_joints[i]))
        return "GoToJointConfig: target_joints[" + std::to_string(i) + "] is not finite";
    return std::nullopt;
  }

  void start_plan(const Action::Goal& goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan_to_joints(
        std::vector<double>(goal.target_joints.begin(), goal.target_joints.end()),
        std::move(on_fb), std::move(on_done));
  }
};

}  // namespace kinova_arm_ros2
