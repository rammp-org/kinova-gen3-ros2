#include "kinova_gen3_ros2/link_server.h"
#include <chrono>
#include <thread>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
namespace kinova_gen3_ros2 {

LinkServer::LinkServer(rclcpp::Node::SharedPtr node, kinova::ArmLink &link,
                       std::string arm, const std::atomic<bool> &stop,
                       std::function<bool()> clear_contact)
    : node_(std::move(node)), link_(link), arm_(std::move(arm)), stop_(stop),
      clear_contact_(std::move(clear_contact)) {
  // Its OWN mutually-exclusive group: on_clear waits up to kClearWait for the
  // arm to re-enter low-level, and must not hold up /estop, the ownership
  // services or a trajectory cancel on the default group meanwhile.
  clear_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  clear_srv_ = node_->create_service<Trigger>(
      "clear_faults",
      std::bind(&LinkServer::on_clear, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, clear_group_);
  // Its own Updater, as GripperServer and Ros2Backend each own one.
  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("kinova_gen3");
  updater_->add("Arm link", this, &LinkServer::diagnostics);
}

void link_diagnostics(const kinova::LinkStatus &s, const std::string &arm,
                      diagnostic_updater::DiagnosticStatusWrapper &stat) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  switch (s.state) {
  case kinova::LinkState::kRunning:
    stat.summary(DiagnosticStatus::OK, "running");
    break;
  case kinova::LinkState::kFaulted:
    stat.summary(DiagnosticStatus::ERROR,
                 "faulted, waiting for /clear_faults: " + s.reason);
    break;
  case kinova::LinkState::kConnecting:
    stat.summary(DiagnosticStatus::STALE,
                 "waiting for the arm at " + arm +
                     (s.reason.empty() ? std::string() : ": " + s.reason));
    break;
  }
  stat.add("link_state", kinova::to_string(s.state));
  stat.add("reason", s.reason);
  stat.add("faults", s.faults);
  stat.add("reconnects", s.reconnects);
}

void LinkServer::diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  link_diagnostics(link_.status(), arm_, stat);
}

void LinkServer::on_clear(const std::shared_ptr<Trigger::Request>,
                          std::shared_ptr<Trigger::Response> res) {
  // One call clears whatever is latched: the contact stop first (it is not a
  // link state, and it survives faults, so a faulted link may also be
  // latched), then the link if it is faulted.
  const bool contact_cleared = clear_contact_ && clear_contact_();
  const std::string contact_msg = contact_cleared ? "contact stop cleared" : "";
  const uint64_t before = link_.status().clears_done;
  if (!link_.request_clear()) {
    if (contact_cleared) {
      res->success = true;
      res->message = contact_msg;
      RCLCPP_INFO(node_->get_logger(), "/clear_faults: %s",
                  res->message.c_str());
      return;
    }
    const auto s = link_.status();
    res->success = false;
    res->message =
        std::string("not faulted (") + kinova::to_string(s.state) + ")";
    return;
  }
  // The clear runs on the link's thread; clears_done moves with the state it
  // resolved to (running, or still faulted and why).
  const auto deadline = std::chrono::steady_clock::now() + kClearWait;
  kinova::LinkStatus s = link_.status();
  while (s.clears_done == before && !stop_.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    s = link_.status();
  }
  res->success = s.state == kinova::LinkState::kRunning;
  if (s.clears_done == before && stop_.load())
    res->message = "stopping";
  else if (s.clears_done == before)
    res->message = "clear requested; no outcome within " +
                   std::to_string(kClearWait.count()) + " s";
  else
    res->message = std::string(kinova::to_string(s.state)) +
                   (s.reason.empty() ? std::string() : ": " + s.reason);
  if (contact_cleared)
    res->message = contact_msg + "; " + res->message;
  RCLCPP_INFO(node_->get_logger(), "/clear_faults: %s", res->message.c_str());
}
} // namespace kinova_gen3_ros2
