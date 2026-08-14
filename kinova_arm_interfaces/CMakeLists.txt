cmake_minimum_required(VERSION 3.8)
project(kinova_arm_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointTrajectory.action"
  "action/GoToEEPose.action"
  "msg/JointImpedanceGains.msg"
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs control_msgs action_msgs geometry_msgs)
ament_package()
