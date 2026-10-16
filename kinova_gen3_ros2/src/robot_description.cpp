// kinova_gen3_ros2/src/robot_description.cpp
#include "kinova_gen3_ros2/robot_description.h"
#include <stdexcept>
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"
namespace kinova_gen3_ros2 {

std::optional<std::string>
wait_for_robot_description(const rclcpp::Node::SharedPtr &node,
                           std::chrono::milliseconds timeout) {
  std::optional<std::string> xml;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      kRobotDescriptionTopic, rclcpp::QoS(1).reliable().transient_local(),
      [&xml](std_msgs::msg::String::ConstSharedPtr m) { xml = m->data; });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!xml && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    ex.spin_once(std::chrono::milliseconds(50));
  ex.remove_node(node);
  return xml;
}

std::vector<std::string>
joint_state_names(const std::string &urdf_xml,
                  std::vector<std::string> arm_joints) {
  urdf::Model model;
  if (!model.initString(urdf_xml))
    throw std::runtime_error(std::string(kRobotDescriptionTopic) +
                             " is not a parseable URDF");
  if (!model.getJoint(kGripperJoint))
    throw std::runtime_error(
        std::string(kRobotDescriptionTopic) + " has no joint '" +
        kGripperJoint + "': is arm_state_publisher serving the arm_ model?");
  arm_joints.emplace_back(kGripperJoint);
  return arm_joints;
}
} // namespace kinova_gen3_ros2
