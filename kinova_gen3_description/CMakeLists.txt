cmake_minimum_required(VERSION 3.8)
project(kinova_gen3_description)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The model is rendered at LAUNCH, from the robot's mount file, by
# kinova_gen3_render.py -- never at build time. A build-time render would bake in
# one mount and leave a flat URDF in the image for anything to pick up by mistake;
# on a tilted arm that silently applies gravity 30 degrees off.
install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
# Importable (description.launch.py does `from kinova_gen3_render import ...`)...
ament_python_install_module(scripts/kinova_gen3_render.py)
# ...and runnable, for the bench: the driver's standalone apps take --urdf.
install(PROGRAMS scripts/kinova_gen3_render.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # PYTEST_DISABLE_PLUGIN_AUTOLOAD: a stale anyio plugin in ~/.local on the Jetson
  # breaks pytest's entrypoint autoload outright (ModuleNotFoundError: _pytest.scope).
  # APPEND_ENV, not ENV, for the paths: the tests need ROS's own python path (xacro,
  # ament_index_python) as well as pinocchio's, and this package's install prefix so
  # get_package_share_directory finds the xacro the render reads.
  set(_test_env
    APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:/usr/local/lib/python3.10/dist-packages
      AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    ENV
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(model_parity test/test_model_parity.py
    ${_test_env}
    KINOVA_OLD_URDF=${CMAKE_CURRENT_SOURCE_DIR}/../../kinova-gen3-driver/models/gen3_7dof_2f85.urdf)
  ament_add_pytest_test(description_log test/test_description_log.py ${_test_env})
  add_launch_test(test/test_description_launch.py TARGET description_launch ${_test_env})
endif()

ament_package()
