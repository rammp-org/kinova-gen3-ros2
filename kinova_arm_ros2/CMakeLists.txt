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
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

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

# The high-level "go to a goal" action servers. Header-only: the lifecycle is
# the PlannedMoveServer<ActionT> template and each concrete server is a thin
# subclass, so there is no translation unit of its own to compile.
add_library(planned_move_servers INTERFACE)
target_include_directories(planned_move_servers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(planned_move_servers INTERFACE
  rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(planned_move_servers INTERFACE
  goal_router curobo_plan_client message_mapping kinova_lowlevel::kinova_lowlevel)

add_library(ros2_backend src/ros2_backend.cpp)
target_include_directories(ros2_backend PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ros2_backend rclcpp rclcpp_action kinova_arm_interfaces sensor_msgs
  geometry_msgs diagnostic_msgs diagnostic_updater)
target_link_libraries(ros2_backend message_mapping kinova_lowlevel::kinova_lowlevel)

# The ROS face of core's ArbitrationSink: ownership services, /estop, /control_status
# and the REP 107 /diagnostics contribution. Holds only an ArbitrationSink&.
add_library(arbitration_server src/arbitration_server.cpp)
target_include_directories(arbitration_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(arbitration_server
  rclcpp kinova_arm_interfaces std_msgs diagnostic_msgs diagnostic_updater)
target_link_libraries(arbitration_server kinova_lowlevel::kinova_lowlevel)

# The ROS face of core's StreamSink: session services, setpoint topics, /stream_status.
add_library(stream_server src/stream_server.cpp)
target_include_directories(stream_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(stream_server rclcpp kinova_arm_interfaces geometry_msgs std_msgs)
target_link_libraries(stream_server kinova_lowlevel::kinova_lowlevel)

# The ROS face of core's GripperSink: /setpoint/gripper in, /gripper_state out, plus the
# REP 107 /diagnostics contribution. Holds only a GripperSink&.
add_library(gripper_server src/gripper_server.cpp)
target_include_directories(gripper_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(gripper_server
  rclcpp kinova_arm_interfaces diagnostic_msgs diagnostic_updater)
target_link_libraries(gripper_server message_mapping kinova_lowlevel::kinova_lowlevel)

add_executable(kinova_arm_node src/bringup_node.cpp)
ament_target_dependencies(kinova_arm_node rclcpp rclcpp_action kinova_arm_interfaces geometry_msgs)
target_link_libraries(kinova_arm_node
  ros2_backend planned_move_servers goal_router curobo_plan_client message_mapping
  arbitration_server stream_server gripper_server kinova_lowlevel::kinova_lowlevel)
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

  # Proves the two failures this tier exists to prevent: a cancel silently refused
  # under kEnforced, and an e-stop that cannot get past a busy arbiter.
  ament_add_gtest(arbitration_integration_test test/arbitration_integration_test.cpp)
  target_link_libraries(arbitration_integration_test kinova_lowlevel::kinova_lowlevel)

  ament_add_gtest(stream_server_test test/stream_server_test.cpp)
  target_include_directories(stream_server_test PRIVATE test)
  target_link_libraries(stream_server_test stream_server)
  ament_target_dependencies(stream_server_test rclcpp kinova_arm_interfaces geometry_msgs)

  ament_add_gtest(gripper_server_test test/gripper_server_test.cpp)
  target_include_directories(gripper_server_test PRIVATE test)
  target_link_libraries(gripper_server_test gripper_server ros2_backend)
  ament_target_dependencies(gripper_server_test
    rclcpp kinova_arm_interfaces sensor_msgs diagnostic_msgs diagnostic_updater)

  ament_add_gtest(arbitration_server_test test/arbitration_server_test.cpp)
  target_include_directories(arbitration_server_test PRIVATE test)
  target_link_libraries(arbitration_server_test arbitration_server)
  ament_target_dependencies(arbitration_server_test
    rclcpp kinova_arm_interfaces diagnostic_msgs diagnostic_updater)

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
    planned_move_servers goal_router curobo_plan_client message_mapping)
  ament_target_dependencies(goto_ee_pose_integration_test
    rclcpp rclcpp_action kinova_arm_interfaces rammp_curobo_interfaces geometry_msgs)

  ament_add_gtest(goto_joint_config_integration_test
    test/goto_joint_config_integration_test.cpp)
  target_include_directories(goto_joint_config_integration_test PRIVATE test)
  target_link_libraries(goto_joint_config_integration_test
    planned_move_servers goal_router curobo_plan_client message_mapping)
  ament_target_dependencies(goto_joint_config_integration_test
    rclcpp rclcpp_action kinova_arm_interfaces rammp_curobo_interfaces geometry_msgs)

  ament_add_gtest(goto_preset_integration_test test/goto_preset_integration_test.cpp)
  target_include_directories(goto_preset_integration_test PRIVATE test)
  target_link_libraries(goto_preset_integration_test
    planned_move_servers goal_router curobo_plan_client message_mapping)
  ament_target_dependencies(goto_preset_integration_test
    rclcpp rclcpp_action kinova_arm_interfaces rammp_curobo_interfaces geometry_msgs)
endif()

ament_package()
