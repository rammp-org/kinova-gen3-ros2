#pragma once
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "kinova_gen3_interfaces/action/go_to_preset.hpp"
#include "kinova_gen3_ros2/planned_move_server.h"
namespace kinova_gen3_ros2 {

// Hosts GoToPreset: a named joint configuration resolved from a registry, then
// planned collision-free by cuRobo exactly as GoToJointConfig is. The registry
// is injected rather than read from ROS params here, so it is testable without
// a parameter server; bringup_node builds it from preset_names / presets.<name>.
class GoToPresetServer : public PlannedMoveServer<kinova_gen3_interfaces::action::GoToPreset> {
 public:
  using Action = kinova_gen3_interfaces::action::GoToPreset;
  using Registry = std::map<std::string, std::vector<double>>;

  GoToPresetServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                   CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb_group,
                   Registry registry)
      : PlannedMoveServer<Action>(node, "go_to_preset", router, planner, cb_group),
        registry_(std::move(registry)) {}

 protected:
  std::optional<std::string> validate(const Action::Goal& goal) override {
    if (!registry_.count(goal.preset_name))   // fail loud; never fall back to a default pose
      return "GoToPreset: unknown preset '" + goal.preset_name + "'";
    return std::nullopt;
  }

  void start_plan(const Action::Goal& goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan_to_joints(registry_.at(goal.preset_name), this->start_config(),
                            std::move(on_fb), std::move(on_done));
  }

 private:
  Registry registry_;
};

}  // namespace kinova_gen3_ros2
