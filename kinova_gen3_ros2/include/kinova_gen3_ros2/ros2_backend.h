// kinova_gen3_ros2/include/kinova_gen3_ros2/ros2_backend.h
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <atomic>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinova_gen3_interfaces/msg/ee_state.hpp"
#include "kinova_gen3_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_gen3_ros2/message_mapping.h"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

class Ros2Backend : public kinova::interface::ActionServerPort,
                    public kinova::interface::StreamPort {
 public:
  using Action = kinova_gen3_interfaces::action::ExecuteJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  explicit Ros2Backend(rclcpp::Node::SharedPtr node);
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }
  // Null until bringup wires it -- and null is a real configuration, not an omission:
  // a robot with no gripper publishes seven joints exactly as before this tier.
  void set_gripper_sink(kinova::interface::GripperSink* sink) { gripper_ = sink; }

  // ActionServerPort (called by the supervisor sampler thread):
  void publish_feedback(const kinova::interface::GoalId&, const kinova::interface::TrajectoryFeedback&) override;
  void settle(const kinova::interface::GoalId&, const kinova::interface::TrajectoryResult&) override;
  // StreamPort (called by the supervisor pump thread):
  void publish_state(const kinova::interface::ArmState&) override;

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
  void handle_accepted(std::shared_ptr<GoalHandle>);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  // /ee_state: the Cartesian sibling of /joint_states. Core hands us ee_pose and
  // ee_twist on every ArmState and we used to drop both, so a client streaming EE
  // poses had no way to read where the tool actually was.
  rclcpp::Publisher<kinova_gen3_interfaces::msg::EeState>::SharedPtr ee_pub_;

  // REP 107 hardware health. ArmState carries the arm's fault flag and we dropped
  // that too -- the arm could be faulted with nothing on the ROS surface saying so.
  // Atomics because publish_state runs on the pump thread and the updater on a ROS
  // timer; there is no state here worth a mutex.
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  std::atomic<bool>   fault_{false};
  std::atomic<double> arm_stamp_s_{0.0};
  std::atomic<bool>   ever_published_{false};
  kinova::interface::CommandSink* sink_ = nullptr;
  kinova::interface::GripperSink* gripper_ = nullptr;
  std::mutex m_;
  // The goal's token, kept so cancel can replay it. A ROS action cancel carries no
  // payload (action_msgs/CancelGoal is one GoalInfo), so without this a cancel would
  // reach the Arbiter with a zero token and be REFUSED under kEnforced -- the motion
  // would keep running while the client believed it had cancelled.
  struct Entry { std::shared_ptr<GoalHandle> gh; kinova::interface::Token token{}; };
  std::map<kinova::interface::GoalId, Entry> handles_;   // GoalId == GoalUUID
};
}  // namespace kinova_gen3_ros2
