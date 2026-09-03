// kinova_gen3_ros2/include/kinova_gen3_ros2/stream_server.h
#pragma once
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "kinova_gen3_interfaces/msg/controller_capability.hpp"
#include "kinova_gen3_interfaces/msg/joint_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/pose_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/stream_status.hpp"
#include "kinova_gen3_interfaces/msg/twist_setpoint.hpp"
#include "kinova_gen3_interfaces/msg/wrench_setpoint.hpp"
#include "kinova_gen3_interfaces/srv/close_stream.hpp"
#include "kinova_gen3_interfaces/srv/list_controllers.hpp"
#include "kinova_gen3_interfaces/srv/open_stream.hpp"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// The ROS face of core's StreamSink: session services, the setpoint topics, and
// /stream_status.
//
// Holds a StreamSink& and NOTHING else from core, so it unit-tests against a fake with
// no robot -- the same reason ArbitrationServer holds only an ArbitrationSink&.
//
// A client names a CONTROLLER; the driver replies with the CHANNELS to publish on. Core
// models a session as a (SetpointKind, ControlModeKind) pair, which would make 20
// combinations representable when 5 are legal; the registry below is the only place
// that collapse lives.
class StreamServer {
 public:
  using OpenStream      = kinova_gen3_interfaces::srv::OpenStream;
  using CloseStream     = kinova_gen3_interfaces::srv::CloseStream;
  using ListControllers = kinova_gen3_interfaces::srv::ListControllers;
  using StreamStatusMsg = kinova_gen3_interfaces::msg::StreamStatus;
  using JointSetpointMsg  = kinova_gen3_interfaces::msg::JointSetpoint;
  using PoseSetpointMsg   = kinova_gen3_interfaces::msg::PoseSetpoint;
  using TwistSetpointMsg  = kinova_gen3_interfaces::msg::TwistSetpoint;
  using WrenchSetpointMsg = kinova_gen3_interfaces::msg::WrenchSetpoint;

  StreamServer(rclcpp::Node::SharedPtr node, kinova::interface::StreamSink& sink);

  // Publish only if core's stream state differs from what was last sent. Called by the
  // session handlers and by a 10 Hz timer, so a teardown core did on its own (deadline
  // expiry, halt) still surfaces.
  void publish_status_if_changed();

  // One controller: a control law plus the channels it reads. `core_backed` is false
  // when core has no SetpointKind for it at all, so there is nothing to ask
  // pair_supported() about.
  struct ControllerRow {
    std::string name;
    std::vector<std::string> channels;
    bool core_backed;
    kinova::interface::SetpointKind    kind;   // meaningful iff core_backed
    kinova::interface::ControlModeKind mode;   // meaningful iff core_backed
  };
  static const std::vector<ControllerRow>& registry();
  // Computed live from core, never declared: a controller is openable only if core has
  // a kind for it, that pair is supported, and it needs just one channel (core admits
  // exactly one SetpointKind per session).
  static bool available(const ControllerRow&);

 private:
  void on_open(const std::shared_ptr<OpenStream::Request>,
               std::shared_ptr<OpenStream::Response>);
  void on_close(const std::shared_ptr<CloseStream::Request>,
                std::shared_ptr<CloseStream::Response>);
  void on_list(const std::shared_ptr<ListControllers::Request>,
               std::shared_ptr<ListControllers::Response>);

  void on_joint_position(const JointSetpointMsg::SharedPtr);
  void on_joint_velocity(const JointSetpointMsg::SharedPtr);
  void on_joint_torque(const JointSetpointMsg::SharedPtr);
  void on_pose(const PoseSetpointMsg::SharedPtr);
  void on_twist(const TwistSetpointMsg::SharedPtr);
  void on_wrench(const WrenchSetpointMsg::SharedPtr);

  rclcpp::Node::SharedPtr node_;
  kinova::interface::StreamSink& sink_;

  rclcpp::Service<OpenStream>::SharedPtr open_srv_;
  rclcpp::Service<CloseStream>::SharedPtr close_srv_;
  rclcpp::Service<ListControllers>::SharedPtr list_srv_;
  rclcpp::Publisher<StreamStatusMsg>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr session_group_;
  rclcpp::CallbackGroup::SharedPtr setpoint_group_;
  rclcpp::Subscription<JointSetpointMsg>::SharedPtr jp_sub_, jv_sub_, jt_sub_;
  rclcpp::Subscription<PoseSetpointMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<TwistSetpointMsg>::SharedPtr twist_sub_;
  rclcpp::Subscription<WrenchSetpointMsg>::SharedPtr wrench_sub_;

  std::mutex m_;
  std::string open_controller_;              // our label for core's (kind, mode)
  std::optional<StreamStatusMsg> last_published_;
};
}  // namespace kinova_gen3_ros2
