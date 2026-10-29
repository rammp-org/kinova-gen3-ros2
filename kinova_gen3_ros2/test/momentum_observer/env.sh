# source this before any script here: host-side ROS 2 that sees the containers (DDS on loopback)
source /opt/ros/humble/setup.bash
source ~/rammp_deps_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/abra/.config/rammp-bench/cyclonedds-local.xml
export ROS_DOMAIN_ID=0
export KINOVA_URDF=$HOME/momentum_observer/arm.urdf
