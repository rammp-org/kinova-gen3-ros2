#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter's StreamSink side. Records what StreamServer
// delegates and lets a test dictate what on_query_stream reports, so status can
// be exercised without a Supervisor, a URDF or threads.
struct FakeStreamSink : public kinova::interface::StreamSink {
  mutable std::mutex m;
  std::vector<std::string> calls;
  kinova::interface::StreamOpenRequest last_open{};
  kinova::interface::Token last_token{};
  kinova::JointVec last_values = kinova::JointVec::Zero();
  kinova::interface::StreamStatus status{};
  bool accept_open = true;

  void note(const std::string &s) {
    std::lock_guard<std::mutex> l(m);
    calls.push_back(s);
  }
  std::vector<std::string> log() const {
    std::lock_guard<std::mutex> l(m);
    return calls;
  }

  kinova::interface::StreamOpenResult
  on_stream_open(const kinova::interface::StreamOpenRequest &r) override {
    note("open");
    {
      std::lock_guard<std::mutex> l(m);
      last_open = r;
    }
    if (!accept_open)
      return {false, kinova::interface::result_code::kStreamRejected,
              "refused by fake"};
    return {true, 0, ""};
  }
  void
  on_stream_close(const kinova::interface::StreamCloseRequest &r) override {
    note("close");
    std::lock_guard<std::mutex> l(m);
    last_token = r.token;
  }
  void on_setpoint_joint_position(
      const kinova::interface::JointSetpoint &s) override {
    note("joint_position");
    std::lock_guard<std::mutex> l(m);
    last_token = s.token;
    last_values = s.values;
  }
  void on_setpoint_joint_velocity(
      const kinova::interface::JointSetpoint &s) override {
    note("joint_velocity");
    std::lock_guard<std::mutex> l(m);
    last_token = s.token;
    last_values = s.values;
  }
  void
  on_setpoint_joint_torque(const kinova::interface::JointSetpoint &s) override {
    note("joint_torque");
    std::lock_guard<std::mutex> l(m);
    last_token = s.token;
    last_values = s.values;
  }
  void on_setpoint_pose(const kinova::interface::PoseSetpoint &s) override {
    note("pose");
    std::lock_guard<std::mutex> l(m);
    last_token = s.token;
  }
  void on_setpoint_twist(const kinova::interface::TwistSetpoint &s) override {
    note("twist");
    std::lock_guard<std::mutex> l(m);
    last_token = s.token;
  }
  kinova::interface::StreamStatus on_query_stream() override {
    std::lock_guard<std::mutex> l(m);
    return status;
  }
};
