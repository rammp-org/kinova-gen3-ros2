#!/usr/bin/env bash
# Swap the arm node between sheppy's container and the feedback-recorder image.
#
#   arm_recorder.sh start    stop sheppy's arm container, run the recorder image
#   arm_recorder.sh stop     stop the recorder container, sheppy restart arm
#   arm_recorder.sh status   which one is up
#
# Nothing in rammp-deployments is edited. The recorder container runs with the
# same flags as the manifest's kinova_gen3_driver alternative (host net, host
# ipc, SYS_NICE, rtprio/memlock, loopback Cyclone config, RT loop on core 11)
# plus a volume for the CSVs. Image: kinova-gen3-ros2:feedback-recorder, built
# from the feat/feedback-recorder branch (v1.1.0 + the recorder).
set -euo pipefail

IMAGE=${IMAGE:-kinova-gen3-ros2:feedback-recorder}
NAME=momentum-arm
SHEPPY_DIR=${SHEPPY_DIR:-$HOME/rammp-deployments/december_2026}
RUNS=${RUNS:-$HOME/momentum_observer/runs}
ARM_IP=${ARM_IP:-192.168.1.10}
CYCLONE=/home/abra/.config/rammp-bench/cyclonedds-local.xml
NODE=/module_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node
URDF=/module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf

running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

case "${1:-status}" in
  start)
    if running "$NAME"; then echo "$NAME already running"; exit 0; fi
    if running sheppy-arm; then
      echo "stopping sheppy-arm (sheppy will show it as exited; 'arm_recorder.sh stop' restarts it)"
      docker stop -t 20 sheppy-arm >/dev/null
    fi
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    mkdir -p "$RUNS"
    docker run -d --name "$NAME" --stop-timeout 20 \
      --network host --ipc host --cap-add SYS_NICE \
      --ulimit rtprio=99 --ulimit memlock=-1 \
      -v "$CYCLONE:$CYCLONE:ro" -v "$RUNS:/runs" \
      -e CYCLONEDDS_URI="file://$CYCLONE" -e ROS_DOMAIN_ID=0 \
      "$IMAGE" "$NODE" --ip "$ARM_IP" --urdf "$URDF" --cpu 11 >/dev/null
    echo "$NAME started from $IMAGE"
    ;;
  stop)
    if running "$NAME"; then docker stop -t 20 "$NAME" >/dev/null; docker rm "$NAME" >/dev/null; echo "$NAME stopped"; fi
    if ! running sheppy-arm; then (cd "$SHEPPY_DIR" && sheppy restart arm); fi
    ;;
  status)
    if running "$NAME"; then echo "recorder"; elif running sheppy-arm; then echo "sheppy"; else echo "none"; fi
    ;;
  logs)
    docker logs --tail "${2:-30}" "$NAME"
    ;;
  *) echo "usage: $0 start|stop|status|logs"; exit 2 ;;
esac
