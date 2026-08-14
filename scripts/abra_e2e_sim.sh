# scripts/abra_e2e_sim.sh — run ON abra: launch node (sim), run the client, assert.
#!/usr/bin/env bash
set -o pipefail
# ROS humble setup.bash references $AMENT_TRACE_SETUP_FILES unguarded, which
# trips set -u — source it (and the workspace overlay) before turning -u on.
source /opt/ros/humble/setup.bash
source /tmp/kinova-ros2-ws/install/setup.bash
set -u
cd /tmp/kinova-ros2-ws/src/kinova-gen3-driver
ros2 run kinova_arm_ros2 kinova_arm_node --sim --urdf models/gen3_7dof_2f85.urdf & NODE=$!
sleep 3
python3 /tmp/kinova-ros2-ws/src/kinova_arm_ros2/kinova_arm_ros2/test/send_trajectory.py --mode position --delta 0.05 --dur 0.4 --expect 0
R1=$?
python3 /tmp/kinova-ros2-ws/src/kinova_arm_ros2/kinova_arm_ros2/test/send_trajectory.py --delta 0.5 --dur 2.0 --path-tol 0.2 --expect -4
R2=$?
# `ros2 run` forks the actual node binary as a *child* of a Python wrapper —
# $NODE is the wrapper's pid, so killing only it orphans the real-time node
# (leaks forever, since it never gets a signal). Reap the wrapper, then kill
# the actual binary by path so it doesn't linger on shared abra.
kill $NODE 2>/dev/null; wait $NODE 2>/dev/null
pkill -TERM -f '/tmp/kinova-ros2-ws/install/kinova_arm_ros2/lib/kinova_arm_ros2/kinova_arm_node' 2>/dev/null
sleep 1
echo "success_case=$R1 divergence_case=$R2"
[ $R1 -eq 0 ] && [ $R2 -eq 0 ]
