#pragma once
// FeedbackRecorder -- every 1 kHz JointFeedback to a CSV, on demand.
//
// A Transport decorator in the same spirit as core's FeedbackTap: it sits in the
// RT loop's call path and, while armed, copies each feedback frame into a
// single-producer / single-consumer ring. A drain thread formats and writes the
// CSV. The RT side does no allocation, no locking, no I/O. When the ring is
// full (the drain thread fell behind) the frame is dropped and counted rather
// than blocking the loop.
//
// Why: /joint_states comes off the Supervisor's non-RT pump at ~100 Hz. The
// momentum-observer work (test/momentum_observer) needs the torque sensors at
// the loop rate, and only this path has them.
//
// Columns match test/momentum_observer/record_joint_states.py so the same
// analysis reads both: t, q1..q7, qd1..qd7, tau1..tau7, fx..tz (NaN here),
// marker (0 here; taps live in a sidecar file keyed by t_wall), then t_wall
// (unix seconds, for merging those markers) and frame_id.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"

namespace kinova_gen3_ros2 {

struct FeedbackSample {
  double t_wall = 0.0;
  uint64_t frame_id = 0;
  double q[kinova::kNumJoints];
  double qd[kinova::kNumJoints];
  double tau[kinova::kNumJoints];
  bool fault = false;
};

// Fixed-capacity SPSC ring. Producer: the RT thread. Consumer: the drain thread.
class SampleRing {
 public:
  explicit SampleRing(size_t capacity_pow2) : buf_(capacity_pow2), mask_(capacity_pow2 - 1) {}
  bool push(const FeedbackSample &s) noexcept {
    const uint64_t head = head_.load(std::memory_order_relaxed);
    const uint64_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= buf_.size()) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buf_[head & mask_] = s;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }
  bool pop(FeedbackSample &out) noexcept {
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    const uint64_t head = head_.load(std::memory_order_acquire);
    if (tail == head) return false;
    out = buf_[tail & mask_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }
  uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

 private:
  std::vector<FeedbackSample> buf_;
  size_t mask_;
  alignas(64) std::atomic<uint64_t> head_{0};
  std::atomic<uint64_t> dropped_{0};
  alignas(64) std::atomic<uint64_t> tail_{0};
};

class FeedbackRecorder : public kinova::Transport {
 public:
  explicit FeedbackRecorder(kinova::Transport &inner, size_t ring_pow2 = 1u << 16)
      : inner_(inner), ring_(ring_pow2) {}
  ~FeedbackRecorder() override { stop(); }

  // ---- Transport: forward everything; tap the feedback-producing calls ----
  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const kinova::ActuatorModes &m) override { inner_.set_actuator_modes(m); }
  void exchange(const kinova::JointCommand &c, kinova::JointFeedback &fb) override {
    inner_.exchange(c, fb);
    tap(fb);
  }
  void send(const kinova::JointCommand &c) override { inner_.send(c); }
  void receive(kinova::JointFeedback &fb) override {
    inner_.receive(fb);
    tap(fb);
  }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

  // ---- Non-RT control. Call from one thread (the ROS service callbacks). ----
  // Opens `path`, writes the header, arms the tap, starts the drain thread.
  // Returns false (and leaves nothing armed) if the file cannot be opened.
  bool start(const std::string &path) {
    stop();
    FILE *f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    std::fputs("t", f);
    for (int i = 1; i <= kinova::kNumJoints; ++i) std::fprintf(f, ",q%d", i);
    for (int i = 1; i <= kinova::kNumJoints; ++i) std::fprintf(f, ",qd%d", i);
    for (int i = 1; i <= kinova::kNumJoints; ++i) std::fprintf(f, ",tau%d", i);
    std::fputs(",fx,fy,fz,tx,ty,tz,marker,t_wall,frame_id\n", f);
    file_ = f;
    path_ = path;
    rows_ = 0;
    t0_ = 0.0;
    // Discard anything still in the ring from a previous session.
    FeedbackSample s;
    while (ring_.pop(s)) {
    }
    running_.store(true, std::memory_order_release);
    armed_.store(true, std::memory_order_release);
    drain_ = std::thread([this] { drain_loop(); });
    return true;
  }
  // Disarms the tap, drains what is left, closes the file. Returns rows written.
  uint64_t stop() {
    if (!running_.load(std::memory_order_acquire)) return rows_;
    armed_.store(false, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    if (drain_.joinable()) drain_.join();
    FeedbackSample s;
    while (ring_.pop(s)) write(s);
    if (file_) {
      std::fclose(file_);
      file_ = nullptr;
    }
    return rows_;
  }
  bool recording() const { return armed_.load(std::memory_order_acquire); }
  uint64_t rows() const { return rows_; }
  uint64_t dropped() const { return ring_.dropped(); }
  const std::string &path() const { return path_; }

 private:
  void tap(const kinova::JointFeedback &fb) noexcept {
    if (!armed_.load(std::memory_order_relaxed)) return;
    FeedbackSample s;
    s.t_wall = std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
    s.frame_id = fb.frame_id;
    s.fault = fb.fault;
    for (int i = 0; i < kinova::kNumJoints; ++i) {
      s.q[i] = fb.q[i];
      s.qd[i] = fb.qd[i];
      s.tau[i] = fb.tau[i];
    }
    ring_.push(s);
  }
  void write(const FeedbackSample &s) {
    if (!file_) return;
    if (rows_ == 0) t0_ = s.t_wall;
    std::fprintf(file_, "%.4f", s.t_wall - t0_);
    for (int i = 0; i < kinova::kNumJoints; ++i) std::fprintf(file_, ",%.6f", s.q[i]);
    for (int i = 0; i < kinova::kNumJoints; ++i) std::fprintf(file_, ",%.6f", s.qd[i]);
    for (int i = 0; i < kinova::kNumJoints; ++i) std::fprintf(file_, ",%.4f", s.tau[i]);
    std::fprintf(file_, ",nan,nan,nan,nan,nan,nan,0,%.6f,%llu\n", s.t_wall,
                 static_cast<unsigned long long>(s.frame_id));
    ++rows_;
  }
  void drain_loop() {
    FeedbackSample s;
    while (running_.load(std::memory_order_acquire)) {
      while (ring_.pop(s)) write(s);
      std::fflush(file_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  kinova::Transport &inner_;
  SampleRing ring_;
  std::atomic<bool> armed_{false};
  std::atomic<bool> running_{false};
  std::thread drain_;
  FILE *file_ = nullptr;
  std::string path_;
  uint64_t rows_ = 0;
  double t0_ = 0.0;
};

}  // namespace kinova_gen3_ros2
