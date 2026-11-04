#pragma once
#include <chrono>
#include <future>
#include <thread>
#include "rclcpp/rclcpp.hpp"

// Spins an executor on a background thread and always cancels + joins it, so an
// early return from a failed ASSERT_* cannot destroy a joinable thread (which
// calls std::terminate and replaces the gtest diagnostic with a bare SIGABRT).
//
// The destructor cancels until spin() has actually returned. In Humble,
// Executor::cancel() clears `spinning` and wakes the wait, but nothing records
// the cancel, and spin() sets `spinning` on entry: a cancel that lands before
// the thread reaches spin() is lost, spin() never returns, and a single
// cancel + join hangs (#69).
class SpinThread {
public:
  explicit SpinThread(rclcpp::Executor &ex)
      : ex_(ex), t_([this] {
          ex_.spin();
          done_.set_value();
        }) {}
  ~SpinThread() {
    do
      ex_.cancel();
    while (finished_.wait_for(std::chrono::milliseconds(10)) !=
           std::future_status::ready);
    t_.join();
  }
  SpinThread(const SpinThread &) = delete;
  SpinThread &operator=(const SpinThread &) = delete;

private:
  rclcpp::Executor &ex_;
  std::promise<void> done_;
  std::future<void> finished_ = done_.get_future();
  std::thread t_; // last: its lambda uses the members above
};
