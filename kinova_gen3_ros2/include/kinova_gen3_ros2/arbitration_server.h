// kinova_gen3_ros2/include/kinova_gen3_ros2/arbitration_server.h
#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "kinova_gen3_interfaces/msg/control_status.hpp"
#include "kinova_gen3_interfaces/msg/e_stop.hpp"
#include "kinova_gen3_interfaces/srv/acquire_control.hpp"
#include "kinova_gen3_interfaces/srv/release_control.hpp"
#include "kinova_gen3_interfaces/srv/revoke_control.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's ArbitrationSink: ownership services, the broadcast
// e-stop, /control_status and the REP 107 /diagnostics contribution.
//
// It holds an ArbitrationSink& and NOTHING else from core -- no Supervisor, no
// ControlMode, no Dynamics -- so it is unit-testable against a fake with no
// robot, no URDF and no threads. That is the same reason the Arbiter itself is
// a decorator rather than part of the Supervisor.
class ArbitrationServer {
public:
  using AcquireControl = kinova_gen3_interfaces::srv::AcquireControl;
  using ReleaseControl = kinova_gen3_interfaces::srv::ReleaseControl;
  using RevokeControl = kinova_gen3_interfaces::srv::RevokeControl;
  using EStop = kinova_gen3_interfaces::msg::EStop;
  using ControlStatus = kinova_gen3_interfaces::msg::ControlStatus;

  ArbitrationServer(rclcpp::Node::SharedPtr node,
                    kinova::interface::ArbitrationSink &arb,
                    const std::string &hardware_id,
                    double estop_clear_max_age_s);

  // Publish only if the arbiter's status differs from what was last sent.
  // Called by every mutating handler (so ownership changes are immediate) and
  // by a 10 Hz timer (so externally-caused changes such as rejected_count are
  // not missed).
  void publish_status_if_changed();

private:
  void on_acquire(const std::shared_ptr<AcquireControl::Request>,
                  std::shared_ptr<AcquireControl::Response>);
  void on_release(const std::shared_ptr<ReleaseControl::Request>,
                  std::shared_ptr<ReleaseControl::Response>);
  void on_revoke(const std::shared_ptr<RevokeControl::Request>,
                 std::shared_ptr<RevokeControl::Response>);
  void on_estop(const EStop::SharedPtr msg);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void forget_token();

  rclcpp::Node::SharedPtr node_;
  kinova::interface::ArbitrationSink &arb_;
  double estop_clear_max_age_s_;

  rclcpp::Service<AcquireControl>::SharedPtr acquire_srv_;
  rclcpp::Service<ReleaseControl>::SharedPtr release_srv_;
  rclcpp::Service<RevokeControl>::SharedPtr revoke_srv_;
  rclcpp::Subscription<EStop>::SharedPtr estop_sub_;
  rclcpp::Publisher<ControlStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr estop_group_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  std::mutex m_;
  kinova::interface::Token
      retained_token_{}; // the token we minted; see on_release
  bool have_retained_ = false;
  std::optional<ControlStatus> last_published_;
};
} // namespace kinova_gen3_ros2
