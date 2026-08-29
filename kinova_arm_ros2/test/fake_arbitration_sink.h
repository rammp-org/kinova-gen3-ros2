#pragma once
#include <mutex>
#include <string>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"

// Stand-in for the Arbiter. Records the calls ControlPlane makes and maintains just
// enough state for status() to be meaningful. No Supervisor, no modes, no robot --
// which is the whole reason ControlPlane holds only an ArbitrationSink&.
struct FakeArbitrationSink : public kinova::interface::ArbitrationSink {
  mutable std::mutex m;
  std::vector<std::string> calls;
  kinova::interface::ArbitrationStatus st{};
  kinova::interface::Token next_token{};
  bool grant_accepted = true;

  void note(const std::string& s) { std::lock_guard<std::mutex> l(m); calls.push_back(s); }
  std::vector<std::string> log() const { std::lock_guard<std::mutex> l(m); return calls; }

  kinova::interface::GrantResult grant(const std::string& owner_id) override {
    note("grant:" + owner_id);
    std::lock_guard<std::mutex> l(m);
    if (!grant_accepted)
      return {false, kinova::interface::Token{}, st.generation, "refused"};
    st.owned = true;
    st.owner_id = owner_id;
    ++st.generation;
    return {true, next_token, st.generation, ""};
  }
  void revoke() override {
    note("revoke");
    std::lock_guard<std::mutex> l(m);
    st.owned = false;
    st.owner_id.clear();
  }
  void estop() override {
    note("estop");
    std::lock_guard<std::mutex> l(m);
    st.estopped = true;
    st.owned = false;
    st.owner_id.clear();
  }
  void estop_clear() override {
    note("estop_clear");
    std::lock_guard<std::mutex> l(m);
    st.estopped = false;
  }
  kinova::interface::ArbitrationStatus status() const override {
    std::lock_guard<std::mutex> l(m);
    return st;
  }
};
