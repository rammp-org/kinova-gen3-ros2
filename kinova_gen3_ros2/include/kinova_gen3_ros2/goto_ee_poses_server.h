#pragma once
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_pose.hpp"
#include "rammp_arm_interfaces/action/go_to_ee_poses.hpp"
#include "kinova_gen3_ros2/plan_legs.h"
#include "kinova_gen3_ros2/planned_move_server.h"
namespace kinova_gen3_ros2 {

// Hosts GoToEEPoses: GoToEEPose through several poses in ONE motion. Each leg
// is planned as GoToEEPose plans it (cuRobo plan_to_pose, with that target's
// hold), from the last point of the leg before it; the legs are blended into
// one trajectory (cuRobo blend) at the targets' speeds and radii and executed
// once. The lifecycle lives in PlannedMoveServer.
class GoToEEPosesServer
    : public PlannedMoveServer<rammp_arm_interfaces::action::GoToEEPoses> {
public:
  using Action = rammp_arm_interfaces::action::GoToEEPoses;
  using Target = rammp_arm_interfaces::msg::EEPoseTarget;

  GoToEEPosesServer(rclcpp::Node::SharedPtr node, GoalRouter &router,
                    CuroboPlanClient &planner,
                    rclcpp::CallbackGroup::SharedPtr cb_group)
      : PlannedMoveServer<Action>(node, "go_to_ee_poses", router, planner,
                                  cb_group) {}

  // The blend carries every target's speed and is executed at 1.0. A
  // top-level speed_scale here would scale it a second time.
  static_assert(!detail::has_speed_scale<Action::Goal>::value,
                "a many-target goal must not carry a top-level speed_scale");

protected:
  // A hold is handed to CuroboPlanClient as GoToEEPose's, which it maps onto
  // the planner by name. EEPoseTarget restates the same three constants.
  using Single = rammp_arm_interfaces::action::GoToEEPose::Goal;
  static_assert(Target::HOLD_NONE == Single::HOLD_NONE &&
                    Target::HOLD_LEVEL == Single::HOLD_LEVEL &&
                    Target::HOLD_FIXED == Single::HOLD_FIXED,
                "EEPoseTarget and GoToEEPose holds must agree");

  std::optional<std::string>
  check_plan(const CuroboPlanClient::Outcome &outcome) override {
    if (auto why =
            pose_plan_frame_rejection(outcome.trajectory.header.frame_id))
      return "GoToEEPoses: " + *why;
    return std::nullopt;
  }

  std::optional<std::string> validate(const Action::Goal &goal) override {
    if (goal.targets.empty())
      return std::string("GoToEEPoses: no targets (at least one is needed)");
    for (size_t k = 0; k < goal.targets.size(); ++k) {
      const Target &t = goal.targets[k];
      const std::string what =
          "GoToEEPoses: targets[" + std::to_string(k) + "]";
      if (auto why = ee_pose_frame_rejection(t.pose.header.frame_id))
        return what + ": " + *why;
      if (auto why = pose_rejection(t.pose.pose))
        return what + ": " + *why;
      if (auto why = speed_scale_rejection(t.speed_scale))
        return what + ": " + *why;
      // An unknown mode is refused, never treated as HOLD_NONE (see
      // GoToEEPose).
      switch (t.orientation_hold) {
      case Target::HOLD_NONE:
      case Target::HOLD_LEVEL:
      case Target::HOLD_FIXED:
        break;
      default:
        return what + ": unknown orientation_hold " +
               std::to_string(t.orientation_hold) +
               " (expected HOLD_NONE=" + std::to_string(Target::HOLD_NONE) +
               ", HOLD_LEVEL=" + std::to_string(Target::HOLD_LEVEL) +
               " or HOLD_FIXED=" + std::to_string(Target::HOLD_FIXED) + ")";
      }
      if (auto why = blend_radius_rejection(t.blend_radius))
        return what + ": " + *why;
    }
    return std::nullopt;
  }

  // Why a target pose cannot be planned to, or nullopt: a NaN would reach the
  // planner as a goal it cannot refuse coherently, and a non-unit quaternion
  // is not an orientation. The tolerance is the planner's own for a hold
  // orientation (|norm - 1| < 1e-3).
  static std::optional<std::string>
  pose_rejection(const geometry_msgs::msg::Pose &p) {
    const auto &x = p.position;
    const auto &q = p.orientation;
    for (const double v : {x.x, x.y, x.z, q.x, q.y, q.z, q.w})
      if (!std::isfinite(v))
        return std::string("pose is not finite");
    const double norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!(std::abs(norm - 1.0) < 1e-3))
      return "pose orientation is not a unit quaternion (norm " +
             std::to_string(norm) + ")";
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
    std::vector<uint8_t> holds;
    std::vector<geometry_msgs::msg::Quaternion> orientations;
    for (size_t k = 0; k < n; ++k) {
      const Target &t = g->targets[k];
      scales.push_back(t.speed_scale);
      holds.push_back(t.orientation_hold);
      orientations.push_back(t.pose.pose.orientation); // what the hold keeps
      if (k + 1 < n) // the last target's radius is not used: it stops
        radii.push_back(t.blend_radius);
    }
    plan_legs(
        n, this->start_config(),
        [this, g](size_t k, std::vector<double> start,
                  CuroboPlanClient::FeedbackCb fb,
                  CuroboPlanClient::DoneCb done) {
          const Target &t = g->targets[k];
          planner_.plan(t.pose.pose, start, t.orientation_hold, std::move(fb),
                        std::move(done));
        },
        [this, radii, scales, holds, orientations](
            const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
            CuroboPlanClient::FeedbackCb fb, CuroboPlanClient::DoneCb done) {
          planner_.blend(legs, radii, scales, holds, orientations,
                         std::move(fb), std::move(done));
        },
        [this, key] { return this->canceling(*key); }, std::move(on_fb),
        std::move(on_done));
  }
};

} // namespace kinova_gen3_ros2
