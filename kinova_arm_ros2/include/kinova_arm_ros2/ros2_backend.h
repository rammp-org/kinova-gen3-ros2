// kinova_arm_ros2/include/kinova_arm_ros2/ros2_backend.h
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinova_arm_interfaces/action/execute_joint_trajectory.hpp"
#include "kinova_arm_ros2/message_mapping.h"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

class Ros2Backend : public kinova::interface::ActionServerPort,
                    public kinova::interface::StreamPort {
 public:
  using Action = kinova_arm_interfaces::action::ExecuteJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  explicit Ros2Backend(rclcpp::Node::SharedPtr node);
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

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
  kinova::interface::CommandSink* sink_ = nullptr;
  std::mutex m_;
  // The goal's token, kept so cancel can replay it. A ROS action cancel carries no
  // payload (action_msgs/CancelGoal is one GoalInfo), so without this a cancel would
  // reach the Arbiter with a zero token and be REFUSED under kEnforced -- the motion
  // would keep running while the client believed it had cancelled.
  struct Entry { std::shared_ptr<GoalHandle> gh; kinova::interface::Token token{}; };
  std::map<kinova::interface::GoalId, Entry> handles_;   // GoalId == GoalUUID
};
}  // namespace kinova_arm_ros2
