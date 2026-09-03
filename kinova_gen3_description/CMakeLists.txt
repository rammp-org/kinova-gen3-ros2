cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_description)
find_package(ament_cmake REQUIRED)

# Expand the xacro at BUILD time, into TWO models. Core's Pinocchio cannot read xacro,
# and expanding at build time (rather than checking the result in) keeps the upstream
# mesh paths -- which resolve to file:///opt/ros/... -- correct for the machine that
# actually built it.
#
#   kinova_gen3.urdf        articulated gripper, 13 DOF -> robot_state_publisher / RViz
#   kinova_gen3_7dof.urdf   gripper frozen,       7 DOF -> the driver
#
# Two, because core's Dynamics asserts nv == 7 and refuses anything else, while RViz
# needs the fingers to move. Freezing keeps the gripper's MASS at the wrist (Pinocchio
# lumps a fixed joint into its parent), so gravity compensation is unaffected.
find_program(XACRO_EXE xacro REQUIRED)
set(MODEL_XACRO   ${CMAKE_CURRENT_SOURCE_DIR}/urdf/kinova_gen3.urdf.xacro)
set(EXPAND        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/expand_model.py)
set(MODEL_URDF    ${CMAKE_CURRENT_BINARY_DIR}/kinova_gen3.urdf)
set(MODEL_7DOF    ${CMAKE_CURRENT_BINARY_DIR}/kinova_gen3_7dof.urdf)

add_custom_command(OUTPUT ${MODEL_URDF}
  COMMAND ${EXPAND} ${MODEL_XACRO} ${MODEL_URDF}
  DEPENDS ${MODEL_XACRO} ${EXPAND}
  COMMENT "Expanding kinova_gen3.urdf (articulated gripper)")
add_custom_command(OUTPUT ${MODEL_7DOF}
  COMMAND ${EXPAND} ${MODEL_XACRO} ${MODEL_7DOF} --freeze-gripper
  DEPENDS ${MODEL_XACRO} ${EXPAND}
  COMMENT "Expanding kinova_gen3_7dof.urdf (gripper frozen, for the driver)")
add_custom_target(kinova_gen3_urdf ALL DEPENDS ${MODEL_URDF} ${MODEL_7DOF})

install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
install(FILES ${MODEL_URDF} ${MODEL_7DOF} DESTINATION share/${PROJECT_NAME}/urdf)
install(PROGRAMS scripts/expand_model.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # PYTEST_DISABLE_PLUGIN_AUTOLOAD: a stale anyio plugin in ~/.local on the Jetson
  # breaks pytest's entrypoint autoload outright (ModuleNotFoundError: _pytest.scope).
  ament_add_pytest_test(model_parity test/test_model_parity.py
    ENV
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      PYTHONPATH=/usr/local/lib/python3.10/dist-packages
      KINOVA_NEW_URDF=${MODEL_URDF}
      KINOVA_7DOF_URDF=${MODEL_7DOF}
      KINOVA_OLD_URDF=${CMAKE_CURRENT_SOURCE_DIR}/../../kinova-gen3-driver/models/gen3_7dof_2f85.urdf)
endif()

ament_package()
