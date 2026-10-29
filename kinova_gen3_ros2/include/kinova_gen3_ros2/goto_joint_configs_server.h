#pragma once
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "rammp_arm_interfaces/action/go_to_joint_configs.hpp"
#include "kinova_gen3_ros2/plan_legs.h"
#include "kinova_gen3_ros2/planned_move_server.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova_gen3_ros2 {

// Hosts GoToJointConfigs: GoToJointConfig through several configurations in
// ONE motion. Each leg is planned as GoToJointConfig plans it (cuRobo
// plan_to_joints), from the last point of the leg before it; the legs are
// blended into one trajectory (cuRobo blend) and executed once. The lifecycle
// lives in PlannedMoveServer.
class GoToJointConfigsServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToJointConfigs> {
public:
  using Action = rammp_arm_interfaces::action::GoToJointConfigs;

  GoToJointConfigsServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                         CuroboPlanClient &planner,
                         rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_joint_configs", router, planner,
                                  cb_group) {}

protected:
  // The blend carries each leg's speed_scale, so it executes as planned.
  double execution_speed_scale(const Action::Goal &) override { return 1.0; }

  std::optional<std::string> validate(const Action::Goal &goal) override {
    const size_t n = goal.targets.size();
    if (n == 0)
      return std::string(
          "GoToJointConfigs: no targets (at least one is needed)");
    // A target is a JointTrajectoryPoint, so unlike GoToJointConfig's
    // float64[7] its width is not type-enforced, and it has fields that mean
    // nothing here. Anything but 7 finite positions is refused, not ignored.
    for (size_t k = 0; k < n; ++k) {
      const auto &p = goal.targets[k];
      const std::string what =
          "GoToJointConfigs: targets[" + std::to_string(k) + "]";
      if (p.positions.size() != static_cast<size_t>(kinova::kNumJoints))
        return what + " has " + std::to_string(p.positions.size()) +
               " positions, expected " + std::to_string(kinova::kNumJoints);
      for (size_t i = 0; i < p.positions.size(); ++i)
        if (!std::isfinite(p.positions[i]))
          return what + ".positions[" + std::to_string(i) + "] is not finite";
      if (!p.velocities.empty() || !p.accelerations.empty() ||
          !p.effort.empty() || p.time_from_start.sec != 0 ||
          p.time_from_start.nanosec != 0)
        return what + ": only positions may be set (velocities, "
                      "accelerations, effort and time_from_start must be "
                      "empty)";
    }
    if (auto why = per_leg_rejection("speed_scale", goal.speed_scale.size(), n))
      return "GoToJointConfigs: " + *why;
    for (const double s : goal.speed_scale)
      if (auto why = speed_scale_rejection(s))
        return "GoToJointConfigs: " + *why;
    if (auto why = blend_radius_rejection(goal.blend_radius, n))
      return "GoToJointConfigs: " + *why;
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    // The legs are planned across several callbacks, so they hold their own
    // copy of the goal.
    auto g = std::make_shared<const Action::Goal>(goal);
    const size_t n = g->targets.size();
    std::vector<double> scales(n);
    for (size_t k = 0; k < n; ++k)
      scales[k] = per_leg(g->speed_scale, k, 1.0);
    const std::vector<double> radii = g->blend_radius.empty()
                                          ? std::vector<double>(n - 1, 0.0)
                                          : g->blend_radius;
    plan_legs(
        n, this->start_config(),
        [this, g](size_t k, std::vector<double> start,
                  CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) {
          planner_.plan_to_joints(g->targets[k].positions, start, std::move(fb),
                                  std::move(done));
        },
        // No holds: these legs are planned in joint space, as GoToJointConfig.
        [this, radii, scales](
            const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
            CuroboPlanClient::FeedbackCb fb, CuroboPlanClient::DoneCb done) {
          planner_.blend(legs, radii, scales, {}, std::move(fb),
                         std::move(done));
        },
        std::move(on_fb), std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
