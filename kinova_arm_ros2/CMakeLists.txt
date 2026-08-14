cmake_minimum_required(VERSION 3.8)
project(kinova_arm_ros2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kinova_arm_interfaces REQUIRED)
find_package(kinova_lowlevel CONFIG REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rammp_curobo_interfaces REQUIRED)

add_library(message_mapping src/message_mapping.cpp)
target_include_directories(message_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(message_mapping rclcpp kinova_arm_interfaces trajectory_msgs)
target_link_libraries(message_mapping kinova_lowlevel::kinova_lowlevel)

add_library(goal_router src/goal_router.cpp)
target_include_directories(goal_router PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(goal_router kinova_lowlevel::kinova_lowlevel)

add_library(curobo_plan_client src/curobo_plan_client.cpp)
target_include_directories(curobo_plan_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(curobo_plan_client
  rclcpp rclcpp_action rammp_curobo_interfaces geometry_msgs trajectory_msgs)

add_library(goto_ee_pose_server src/goto_ee_pose_server.cpp)
target_include_directories(goto_ee_pose_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(goto_ee_pose_server
  rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(goto_ee_pose_server
  goal_router curobo_plan_client message_mapping kinova_lowlevel::kinova_lowlevel)

add_library(ros2_backend src/ros2_backend.cpp)
target_include_directories(ros2_backend PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ros2_backend rclcpp rclcpp_action kinova_arm_interfaces sensor_msgs)
target_link_libraries(ros2_backend message_mapping kinova_lowlevel::kinova_lowlevel)

add_executable(kinova_arm_node src/bringup_node.cpp)
ament_target_dependencies(kinova_arm_node rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(kinova_arm_node
  ros2_backend goto_ee_pose_server goal_router curobo_plan_client message_mapping
  kinova_lowlevel::kinova_lowlevel)
option(KINOVA_ENABLE_KORTEX "Link the real KortexTransport path" OFF)
if(NOT KINOVA_ENABLE_KORTEX)
  target_compile_definitions(kinova_arm_node PRIVATE KINOVA_NO_KORTEX)
endif()
# kinova_lowlevel links pinocchio (shared libs under the cmeel prefix) with no
# RPATH of its own (static lib); without this the installed node can't find
# libpinocchio_*.so at runtime even though it links/builds fine.
set_target_properties(kinova_arm_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS kinova_arm_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ../models DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(message_mapping_test test/message_mapping_test.cpp)
  target_link_libraries(message_mapping_test message_mapping)
  ament_target_dependencies(message_mapping_test kinova_arm_interfaces)

  ament_add_gtest(goal_router_test test/goal_router_test.cpp)
  target_link_libraries(goal_router_test goal_router)

  ament_add_gtest(curobo_plan_client_test test/curobo_plan_client_test.cpp)
  target_include_directories(curobo_plan_client_test PRIVATE test)
  target_link_libraries(curobo_plan_client_test curobo_plan_client)
  ament_target_dependencies(curobo_plan_client_test
    rclcpp rclcpp_action rammp_curobo_interfaces geometry_msgs)

  ament_add_gtest(goto_ee_pose_integration_test test/goto_ee_pose_integration_test.cpp)
  target_include_directories(goto_ee_pose_integration_test PRIVATE test)
  target_link_libraries(goto_ee_pose_integration_test
    goto_ee_pose_server goal_router curobo_plan_client message_mapping)
  ament_target_dependencies(goto_ee_pose_integration_test
    rclcpp rclcpp_action kinova_arm_interfaces rammp_curobo_interfaces geometry_msgs)
endif()

ament_package()
