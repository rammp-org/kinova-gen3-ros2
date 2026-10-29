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

// A per-leg value: empty -> dflt for every leg; one value -> that for every
// leg; otherwise v[k]. per_leg_rejection() has already refused other lengths.
template <class T> T per_leg(const std::vector<T> &v, size_t k, T dflt) {
  if (v.empty())
    return dflt;
  return v.size() == 1 ? v[0] : v[k];
}

// Why a per-leg array of `size` values cannot serve `n` targets, or nullopt.
inline std::optional<std::string> per_leg_rejection(const std::string &field,
                                                    size_t size, size_t n) {
  if (size == 0 || size == 1 || size == n)
    return std::nullopt;
  return field + " has " + std::to_string(size) + " values for " +
         std::to_string(n) + " targets (expected 0, 1 or " + std::to_string(n) +
         ")";
}

// Why this blend_radius cannot serve `n` targets, or nullopt. One per
// junction, so n - 1 of them, or none at all (stop on every target).
inline std::optional<std::string>
blend_radius_rejection(const std::vector<double> &r, size_t n) {
  if (!r.empty() && r.size() != n - 1)
    return "blend_radius has " + std::to_string(r.size()) + " values for " +
           std::to_string(n) + " targets (expected 0 or " +
           std::to_string(n - 1) + ", one per junction)";
  for (size_t i = 0; i < r.size(); ++i) {
    if (!std::isfinite(r[i]) || r[i] < 0.0) {
      char buf[96];
      std::snprintf(buf, sizeof buf,
                    "blend_radius[%zu] must be finite and >= 0; got %g", i,
                    r[i]);
      return std::string(buf);
    }
  }
  return std::nullopt;
}

// Plans leg k (0-based) from `start`; one plan()/plan_to_joints() call.
using PlanLegFn =
    std::function<void(size_t k, std::vector<double> start,
                       CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)>;
// Blends the planned legs; one CuroboPlanClient::blend() call.
using BlendLegsFn = std::function<void(
    const std::vector<trajectory_msgs::msg::JointTrajectory> &legs,
    CuroboPlanClient::FeedbackCb, CuroboPlanClient::DoneCb)>;

namespace detail {
struct LegChain {
  size_t n;
  PlanLegFn plan_leg;
  BlendLegsFn blend;
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

inline void plan_leg(std::shared_ptr<LegChain> c, size_t k,
                     std::vector<double> start) {
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
// blend's outcome or with the first failure: "leg k/N: <reason>" for a leg,
// "blend: <reason>" for the blend, and nothing after it is planned. Planner
// feedback is relayed with the same prefix. The callbacks run wherever the
// planner calls them (see CuroboPlanClient's THREADING note).
inline void plan_legs(size_t n, std::vector<double> start, PlanLegFn plan_leg,
                      BlendLegsFn blend, CuroboPlanClient::FeedbackCb on_fb,
                      CuroboPlanClient::DoneCb on_done) {
  auto c =
      std::make_shared<detail::LegChain>(detail::LegChain{n,
                                                          std::move(plan_leg),
                                                          std::move(blend),
                                                          std::move(on_fb),
                                                          std::move(on_done),
                                                          {}});
  detail::plan_leg(std::move(c), 0, std::move(start));
}

} // namespace kinova_gen3_ros2
