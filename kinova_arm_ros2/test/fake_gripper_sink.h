#pragma once
#include <mutex>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter's GripperSink side. Records what GripperServer delegates and
// lets a test dictate what on_query_gripper reports -- the same shape as
// FakeStreamSink, and for the same reason: no Supervisor, no URDF, no threads.
struct FakeGripperSink : public kinova::interface::GripperSink {
  mutable std::mutex m;
  std::vector<kinova::interface::GripperSetpoint> setpoints;
  kinova::interface::GripperState state{};

  void on_gripper_setpoint(const kinova::interface::GripperSetpoint& s) override {
    std::lock_guard<std::mutex> l(m);
    setpoints.push_back(s);
  }
  kinova::interface::GripperState on_query_gripper() override {
    std::lock_guard<std::mutex> l(m);
    return state;
  }
  std::size_t count() const { std::lock_guard<std::mutex> l(m); return setpoints.size(); }
};
