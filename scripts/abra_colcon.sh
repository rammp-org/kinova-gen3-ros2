#!/usr/bin/env bash
# Deploy loop: rsync muk -> abra colcon workspace, colcon build, optional test.
# Usage: abra_colcon.sh [colcon-args...]   e.g.  abra_colcon.sh --packages-select kinova_lowlevel
set -euo pipefail
CORE_SRC="/home/swapnil/atdev/kinova-gen3-driver/"          # Plan 2 branch working tree
ROS_SRC="/home/swapnil/atdev/kinova_gen3_ros2/"
WS="/tmp/kinova-ros2-ws"
CMEEL="/usr/local/lib/python3.10/dist-packages/cmeel.prefix"

rsync -az --mkpath --delete --exclude '.git/' --exclude 'build/' --exclude 'build_kortex/' --exclude 'site/' \
  "$CORE_SRC" "abra:$WS/src/kinova-gen3-driver/"
rsync -az --mkpath --delete --exclude '.git/' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "$ROS_SRC" "abra:$WS/src/kinova_gen3_ros2/"
rsync -az --mkpath --delete --exclude '.git/' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "/home/swapnil/atdev/RAMMP-CuRobo/rammp_curobo_interfaces/" "abra:$WS/src/rammp_curobo_interfaces/"

ssh abra "bash -lc '
  set -eo pipefail
  # ROS humble setup.bash references \$AMENT_TRACE_SETUP_FILES unguarded, which
  # trips set -u — source it before turning -u on.
  source /opt/ros/humble/setup.bash
  set -u
  export CMAKE_PREFIX_PATH=$CMEEL:\${CMAKE_PREFIX_PATH:-}
  cd $WS
  colcon build --event-handlers console_direct+ $* 2>&1 | tail -40
'"
