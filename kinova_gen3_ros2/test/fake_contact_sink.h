#pragma once
#include <mutex>
#include <vector>
#include "kinova_lowlevel/interface/contact.h"
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Supervisor's contact side. on_set_contact_thresholds
// resolves against the spec's starting profiles with core's own
// resolve_contact, as the Supervisor does, so refusals carry core's text; a
// test dictates what on_query_contact reports. Wrap it in a real Arbiter to
// get the token gate. No modes, no URDF, no threads.
struct FakeContactSink : public kinova::interface::CommandSink {
  using JointVec = kinova::JointVec;
  mutable std::mutex m;
  kinova::interface::ContactProfiles profiles = [] {
    kinova::interface::ContactProfiles p;
    p.sensitive << 4, 4, 3, 3, 1.2, 1.2, 1.2;
    p.normal << 8, 8, 6, 6, 2, 2, 2;
    p.loose << 20, 20, 15, 15, 5, 5, 5;
    p.ceiling << 30, 30, 22, 22, 7, 7, 7;
    return p;
  }();
  kinova::interface::ContactState state = [this] {
    kinova::interface::ContactState s;
    s.enabled = true;
    s.status.phase = kinova::ContactPhase::kArmed;
    s.status.theta = profiles.normal;
    return s;
  }();
  std::vector<kinova::interface::ContactThresholdsRequest> sets; // reached us

  kinova::interface::ContactThresholdsResult on_set_contact_thresholds(
      const kinova::interface::ContactThresholdsRequest &r) override {
    std::lock_guard<std::mutex> l(m);
    sets.push_back(r);
    JointVec theta;
    const auto c = kinova::interface::resolve_contact(r, profiles, theta);
    if (!c.ok)
      return {false, c.message, state.profile, state.status.theta};
    state.profile = r.profile;
    state.status.theta = theta;
    return {true, "", state.profile, theta};
  }
  kinova::interface::ContactState on_query_contact() override {
    std::lock_guard<std::mutex> l(m);
    return state;
  }
  std::size_t set_count() const {
    std::lock_guard<std::mutex> l(m);
    return sets.size();
  }

  // The rest of CommandSink, unused here.
  kinova::interface::GoalResponse
  on_trajectory_goal(const kinova::interface::TrajectoryGoal &) override {
    return kinova::interface::GoalResponse::kAccept;
  }
  void
  on_trajectory_accepted(const kinova::interface::GoalId &,
                         const kinova::interface::TrajectoryGoal &) override {}
  kinova::interface::CancelResponse
  on_trajectory_cancel(const kinova::interface::CancelRequest &) override {
    return kinova::interface::CancelResponse::kAccept;
  }
  kinova::interface::GainsResult
  on_set_gains(const kinova::interface::GainsRequest &) override {
    return {};
  }
  kinova::interface::ArmState on_query_state() override { return {}; }
  void on_halt(kinova::interface::HaltReason) override {}
};
