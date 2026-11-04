#pragma once
#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rammp_arm_interfaces/action/go_to_preset.hpp"
#include "kinova_gen3_ros2/planned_move_server.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_gen3_ros2 {

// Hosts GoToPreset: a named joint configuration resolved from a registry, then
// planned collision-free by cuRobo exactly as GoToJointConfig is. The registry
// is injected rather than read from ROS params here, so it is testable without
// a parameter server; bringup_node builds it from preset_names /
// presets.<name>.
class GoToPresetServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToPreset> {
public:
  using Action = rammp_arm_interfaces::action::GoToPreset;
  using Registry = std::map<std::string, std::vector<double>>;

  GoToPresetServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                   CuroboPlanClient &planner,
                   rclcpp::CallbackGroup::SharedPtr cb_group, Registry registry,
                   std::vector<std::string> arm_joint_names)
      : PlannedMoveServer<Action>(node, "go_to_preset", router, planner,
                                  cb_group, std::move(arm_joint_names)),
        registry_(std::move(registry)) {}

protected:
  std::optional<std::string> validate(const Action::Goal &goal) override {
    // speed_scale is the only new field this action carries. orientation_hold
    // is deliberately absent: this plans in joint space, where holding a tool
    // orientation has no meaning.
    if (auto why = speed_scale_rejection(goal.speed_scale))
      return "GoToPreset: " + *why;
    if (!registry_.count(
            goal.preset_name)) // fail loud; never fall back to a default pose
      return "GoToPreset: unknown preset '" + goal.preset_name + "'";
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    planner_.plan_to_joints(registry_.at(goal.preset_name),
                            this->start_config(), std::move(on_fb),
                            std::move(on_done));
  }

private:
  Registry registry_;
};

// Build the GoToPreset registry from ROS params: `preset_names` lists the
// presets, `presets.<name>` gives each one's 7 joint angles (rad). `home` and
// `rest` are the required names callers code against, so they are always
// registered even when `preset_names` omits them. Defaults: Kinova's factory
// Home, and Kinova's Retract with joint 4 at -135 deg (factory -146 deg
// self-collides in cuRobo). An installation overrides both. A mis-sized entry
// is dropped with a warning rather than padded --
// GoToPreset then rejects that name outright instead of moving the arm to a
// half-specified configuration.
inline GoToPresetServer::Registry load_preset_registry(rclcpp::Node &node) {
  const GoToPresetServer::Registry defaults = {
      {"home", {0.0, 0.262, 3.142, -2.269, 0.0, 0.96, 1.571}},
      {"rest", {0.0, -0.349, 3.142, -2.356, 0.0, -0.873, 1.571}}};
  auto names = node.declare_parameter<std::vector<std::string>>(
      "preset_names", {"home", "rest"});
  for (const auto &d : defaults)
    if (std::find(names.begin(), names.end(), d.first) == names.end())
      names.push_back(d.first);
  GoToPresetServer::Registry reg;
  for (const auto &n : names) {
    const auto it = defaults.find(n);
    const auto q = node.declare_parameter<std::vector<double>>(
        "presets." + n,
        it != defaults.end() ? it->second : std::vector<double>{});
    if (q.size() == static_cast<size_t>(kinova::kNumJoints)) {
      reg[n] = q;
      // The defaults assume a level mount and an empty workspace; say so
      // rather than let an unconfigured cell move there without notice.
      if (it != defaults.end() && q == it->second)
        RCLCPP_WARN(node.get_logger(),
                    "preset '%s' is not configured for this installation - "
                    "using the default pose (level mount)",
                    n.c_str());
    } else {
      RCLCPP_WARN(node.get_logger(),
                  "preset '%s' has %zu joint values (need %d) - skipping",
                  n.c_str(), q.size(), kinova::kNumJoints);
    }
  }
  std::string listed;
  for (const auto &e : reg)
    listed += (listed.empty() ? "" : ", ") + e.first;
  RCLCPP_INFO(node.get_logger(), "presets: %s", listed.c_str());
  return reg;
}

} // namespace kinova_gen3_ros2
