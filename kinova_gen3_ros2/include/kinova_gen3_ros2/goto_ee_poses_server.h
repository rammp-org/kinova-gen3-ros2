#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "rammp_arm_interfaces/action/go_to_ee_pose.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_poses.hpp"
#include "kinova_gen3_ros2/plan_legs.h"
#include "kinova_gen3_ros2/planned_move_server.h"
namespace kinova_gen3_ros2 {

// Hosts GoToEEPoses: GoToEEPose through several poses in ONE motion. Each leg
// is planned as GoToEEPose plans it (cuRobo plan_to_pose, with that leg's
// hold), from the last point of the leg before it; the legs are blended into
// one trajectory (cuRobo blend) and executed once. The lifecycle lives in
// PlannedMoveServer.
class GoToEEPosesServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToEEPoses> {
public:
  using Action = rammp_arm_interfaces::action::GoToEEPoses;

  GoToEEPosesServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                    CuroboPlanClient &planner,
                    rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_ee_poses", router, planner,
                                  cb_group) {}

protected:
  // A hold is handed to CuroboPlanClient as GoToEEPose's, which it maps onto
  // the planner by name. This action restates the same three constants.
  using Single = rammp_arm_interfaces::action::GoToEEPose::Goal;
  static_assert(Action::Goal::HOLD_NONE == Single::HOLD_NONE &&
                    Action::Goal::HOLD_LEVEL == Single::HOLD_LEVEL &&
                    Action::Goal::HOLD_FIXED == Single::HOLD_FIXED,
                "GoToEEPoses and GoToEEPose holds must agree");

  std::optional<std::string>
  check_plan(const CuroboPlanClient::Outcome &outcome) override {
    if (auto why =
            pose_plan_frame_rejection(outcome.trajectory.header.frame_id))
      return "GoToEEPoses: " + *why;
    return std::nullopt;
  }

  // The blend carries each leg's speed_scale, so it executes as planned.
  double execution_speed_scale(const Action::Goal &) override { return 1.0; }

  std::optional<std::string> validate(const Action::Goal &goal) override {
    const size_t n = goal.targets.size();
    if (n == 0)
      return std::string("GoToEEPoses: no targets (at least one is needed)");
    for (size_t k = 0; k < n; ++k)
      if (auto why = ee_pose_frame_rejection(goal.targets[k].header.frame_id))
        return "GoToEEPoses: targets[" + std::to_string(k) + "]: " + *why;
    if (auto why = per_leg_rejection("speed_scale", goal.speed_scale.size(), n))
      return "GoToEEPoses: " + *why;
    for (const double s : goal.speed_scale)
      if (auto why = speed_scale_rejection(s))
        return "GoToEEPoses: " + *why;
    if (auto why = per_leg_rejection("orientation_hold",
                                     goal.orientation_hold.size(), n))
      return "GoToEEPoses: " + *why;
    // An unknown mode is refused, never treated as HOLD_NONE (see GoToEEPose).
    for (const uint8_t h : goal.orientation_hold) {
      switch (h) {
      case Action::Goal::HOLD_NONE:
      case Action::Goal::HOLD_LEVEL:
      case Action::Goal::HOLD_FIXED:
        break;
      default:
        return "GoToEEPoses: unknown orientation_hold " + std::to_string(h) +
               " (expected HOLD_NONE=" +
               std::to_string(Action::Goal::HOLD_NONE) +
               ", HOLD_LEVEL=" + std::to_string(Action::Goal::HOLD_LEVEL) +
               " or HOLD_FIXED=" + std::to_string(Action::Goal::HOLD_FIXED) +
               ")";
      }
    }
    if (auto why = blend_radius_rejection(goal.blend_radius, n))
      return "GoToEEPoses: " + *why;
    return std::nullopt;
  }

  void start_plan(const Action::Goal &goal, CuroboPlanClient::FeedbackCb on_fb,
                  CuroboPlanClient::DoneCb on_done) override {
    // The legs are planned across several callbacks, so they hold their own
    // copy of the goal.
    auto g = std::make_shared<const Action::Goal>(goal);
    const size_t n = g->targets.size();
    std::vector<double> scales(n);
    std::vector<uint8_t> holds(n);
    for (size_t k = 0; k < n; ++k) {
      scales[k] = per_leg(g->speed_scale, k, 1.0);
      holds[k] = per_leg(g->orientation_hold, k,
                         static_cast<uint8_t>(Action::Goal::HOLD_NONE));
    }
    const std::vector<double> radii = g->blend_radius.empty()
                                          ? std::vector<double>(n - 1, 0.0)
                                          : g->blend_radius;
    plan_legs(
        n, this->start_config(),
        [this, g, holds](size_t k, std::vector<double> start,
                         CuroboPlanClient::FeedbackCb fb,
                         CuroboPlanClient::DoneCb done) {
          planner_.plan(g->targets[k].pose, start, holds[k], std::move(fb),
                        std::move(done));
        },
        [this, radii, scales, holds](
            const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
            CuroboPlanClient::FeedbackCb fb, CuroboPlanClient::DoneCb done) {
          planner_.blend(legs, radii, scales, holds, std::move(fb),
                         std::move(done));
        },
        std::move(on_fb), std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
