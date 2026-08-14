cmake_minimum_required(VERSION 3.8)
project(kinova_arm_ros2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kinova_arm_interfaces REQUIRED)
find_package(kinova_lowlevel CONFIG REQUIRED)

add_library(message_mapping src/message_mapping.cpp)
target_include_directories(message_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(message_mapping rclcpp kinova_arm_interfaces)
target_link_libraries(message_mapping kinova_lowlevel::kinova_lowlevel)

add_library(ros2_backend src/ros2_backend.cpp)
target_include_directories(ros2_backend PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ros2_backend rclcpp rclcpp_action kinova_arm_interfaces)
target_link_libraries(ros2_backend message_mapping kinova_lowlevel::kinova_lowlevel)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(message_mapping_test test/message_mapping_test.cpp)
  target_link_libraries(message_mapping_test message_mapping)
  ament_target_dependencies(message_mapping_test kinova_arm_interfaces)
endif()

ament_package()
