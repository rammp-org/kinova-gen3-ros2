#pragma once
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "rammp_arm_interfaces/action/go_to_joint_configs.hpp"
#include "kinova_gen3_ros2/plan_legs.h"
#include "kinova_gen3_ros2/planned_move_server.h"
namespace kinova_gen3_ros2 {

// Hosts GoToJointConfigs: GoToJointConfig through several configurations in
// ONE motion. Each leg is planned as GoToJointConfig plans it (cuRobo
// plan_to_joints), from the last point of the leg before it; the legs are
// blended into one trajectory (cuRobo blend) at the targets' speeds and radii
// and executed once. The lifecycle lives in PlannedMoveServer.
class GoToJointConfigsServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToJointConfigs> {
public:
  using Action = rammp_arm_interfaces::action::GoToJointConfigs;
  using Target = rammp_arm_interfaces::msg::JointConfigTarget;

  GoToJointConfigsServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                         CuroboPlanClient &planner,
                         rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_joint_configs", router, planner,
                                  cb_group) {}

  // The blend carries every target's speed and is executed at 1.0. A
  // top-level speed_scale here would scale it a second time.
  static_assert(!detail::has_speed_scale<Action::Goal>::value,
                "a many-target goal must not carry a top-level speed_scale");

protected:
  std::optional<std::string> validate(const Action::Goal &goal) override {
    if (goal.targets.empty())
      return std::string(
          "GoToJointConfigs: no targets (at least one is needed)");
    for (size_t k = 0; k < goal.targets.size(); ++k) {
      const Target &t = goal.targets[k];
      const std::string what =
          "GoToJointConfigs: targets[" + std::to_string(k) + "]";
      // float64[7]: the width is type-enforced, finiteness is not (as on
      // GoToJointConfig).
      for (size_t i = 0; i < t.target_joints.size(); ++i)
        if (!std::isfinite(t.target_joints[i]))
          return what + ".target_joints[" + std::to_string(i) +
                 "] is not finite";
      if (auto why = speed_scale_rejection(t.speed_scale))
        return what + ": " + *why;
      if (auto why = blend_radius_rejection(t.blend_radius))
        return what + ": " + *why;
    }
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    // The legs are planned across several callbacks, so they hold their own
    // copy of the goal; `&goal` is kept only to ask whether it was canceled.
    auto g = std::make_shared<const Action::Goal>(goal);
    const Action::Goal *key = &goal;
    const size_t n = g->targets.size();
    std::vector<double> scales, radii;
    for (size_t k = 0; k < n; ++k) {
      scales.push_back(g->targets[k].speed_scale);
      if (k + 1 < n) // the last target's radius is not used: it stops
        radii.push_back(g->targets[k].blend_radius);
    }
    plan_legs(
        n, this->start_config(),
        [this, g](size_t k, std::vector<double> start,
                  CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) {
          const auto &q = g->targets[k].target_joints;
          planner_.plan_to_joints(std::vector<double>(q.begin(), q.end()),
                                  start, std::move(fb), std::move(done));
        },
        // No holds: these legs are planned in joint space, as GoToJointConfig.
        [this, radii, scales](
            const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
            CuroboPlanClient::FeedbackCb fb, CuroboPlanClient::DoneCb done) {
          planner_.blend(legs, radii, scales, {}, {}, std::move(fb),
                         std::move(done));
        },
        [this, key] { return this->canceling(*key); }, std::move(on_fb),
        std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
