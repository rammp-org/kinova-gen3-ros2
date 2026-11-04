#pragma once
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "kinova_gen3_ros2/curobo_plan_client.h"
namespace kinova_gen3_ros2 {

// Shared by GoToEEPoses and GoToJointConfigs: several targets in ONE motion.
// Each leg is planned as the single-target action plans it, from the last
// point of the leg before it, and the legs are then blended into one
// trajectory (CuroboPlanClient::blend).

// Why a target's blend_radius is unacceptable, or nullopt. Checked on every
// target, the last included, though the last one's is never used.
inline std::optional<std::string> blend_radius_rejection(double r) {
  if (std::isfinite(r) && r >= 0.0)
    return std::nullopt;
  char buf[64];
  std::snprintf(buf, sizeof buf, "blend_radius must be finite and >= 0; got %g",
                r);
  return std::string(buf);
}

// Plans leg k (0-based) from `start`; one plan()/plan_to_joints() call.
using PlanLegFn =
    std::function<void(size_t k, std::vector<double> start,
                       CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)>;
// Blends the planned legs; one CuroboPlanClient::blend() call.
using BlendLegsFn = std::function<void(
    const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
    CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)>;
// Whether the goal has been canceled (PlannedMoveServer::canceling).
using CancelingFn = std::function<bool()>;

namespace detail {
struct LegChain {
  size_t n;
  PlanLegFn plan_leg;
  BlendLegsFn blend;
  CancelingFn canceling;
  CuroboPlanClient::FeedbackCb on_fb;
  CuroboPlanClient::DoneCb on_done;
  std::vector<trajectory_msgs::msg::JointTrajectory> legs;
};

inline CuroboPlanClient::FeedbackCb prefixed(CuroboPlanClient::FeedbackCb fb,
                                             std::string prefix) {
  return [fb, prefix](const std::string &state) {
    if (fb)
      fb(prefix + state);
  };
}

// planner_.cancel() reaches only the plan in flight, and nothing is in flight
// between two steps of the chain. So the chain asks before each step, and a
// canceled goal ends here, through on_done like any failure.
inline bool stop_if_canceled(const LegChain &c) {
  if (!c.canceling())
    return false;
  c.on_done({false, "canceled during planning", {}, 0.0});
  return true;
}

inline void plan_leg(std::shared_ptr<LegChain> c, size_t k,
                     std::vector<double> start) {
  if (stop_if_canceled(*c))
    return;
  const std::string name =
      "leg " + std::to_string(k + 1) + "/" + std::to_string(c->n) + ": ";
  c->plan_leg(k, std::move(start), prefixed(c->on_fb, name),
              [c, k, name](CuroboPlanClient::Outcome o) {
                // The next leg starts at this one's last point, so a plan
                // without one is a failure of this leg, not something to index
                // into.
                if (o.ok && o.trajectory.points.empty()) {
                  o.ok = false;
                  o.message = "planner returned an empty trajectory";
                }
                if (!o.ok) {
                  o.message = name + o.message;
                  c->on_done(std::move(o));
                  return;
                }
                c->legs.push_back(std::move(o.trajectory));
                if (k + 1 < c->n) {
                  plan_leg(c, k + 1, c->legs.back().points.back().positions);
                  return;
                }
                if (stop_if_canceled(*c))
                  return;
                c->blend(c->legs, prefixed(c->on_fb, "blend: "),
                         [c](CuroboPlanClient::Outcome b) {
                           if (!b.ok)
                             b.message = "blend: " + b.message;
                           c->on_done(std::move(b));
                         });
              });
}
} // namespace detail

// Plans legs 0..n-1, each from the last point of the leg before it (the first
// from `start`), then blends them. on_done fires EXACTLY ONCE, with the
// blend's outcome or with the first failure: "leg k/N: <reason>" for a leg
// (k counted from 1), "blend: <reason>" for the blend, and nothing after it is
// planned. A cancel seen before a leg or the blend ends the chain there, with
// a failed outcome the server settles PREEMPTED. Planner feedback is relayed
// with the same prefix. The callbacks run wherever the planner calls them (see
// CuroboPlanClient's THREADING note).
inline void plan_legs(size_t n, std::vector<double> start, PlanLegFn plan_leg,
                      BlendLegsFn blend, CancelingFn canceling,
                      CuroboPlanClient::FeedbackCb on_fb,
                      CuroboPlanClient::DoneCb on_done) {
  auto c =
      std::make_shared<detail::LegChain>(detail::LegChain{n,
                                                          std::move(plan_leg),
                                                          std::move(blend),
                                                          std::move(canceling),
                                                          std::move(on_fb),
                                                          std::move(on_done),
                                                          {}});
  detail::plan_leg(std::move(c), 0, std::move(start));
}

} // namespace kinova_gen3_ros2
