#!/usr/bin/env bash
# Floor-tier smoke test: the image runs, it publishes, and it exits on SIGTERM.
#
# No --init on purpose. The container runs the way sheppyd will run it, so a
# process that ignores SIGTERM fails here rather than on the robot.
#
# This runs the hardware-free path only: --network host and ROS_DOMAIN_ID are
# hardcoded below and the fragment's `container:` block (devices, gpus, etc.)
# is deliberately not read. A module that needs a device or a GPU to run at
# all should point this script at its mock alternative's command instead of
# expecting a CI runner to have that hardware.
set -euo pipefail

# The node's startup banner. It also proves the SIGTERM path below is real: the
# Dockerfile execs the node binary DIRECTLY so it is PID 1 and its handler runs
# safe_shutdown(). `ros2 run` would fork it under a Python wrapper and the signal
PATTERN="${SMOKE_PATTERN:-kinova_gen3_node up}"

IMAGE="${1:-kinova-gen3-ros2:humble}"
NAME="rammp-smoke-$$"

docker run -d --rm --name "$NAME" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT

sleep 5

if ! docker logs "$NAME" 2>&1 | grep -q "$PATTERN"; then
  echo "FAIL: no match for pattern '$PATTERN' in the container logs"
  docker logs "$NAME" 2>&1 | tail -20
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' "$NAME")" != "true" ]; then
  echo "FAIL: container exited on its own before SIGTERM was sent"
  echo "      A single heartbeat followed by a crash is not a passing module."
  docker logs "$NAME" 2>&1 | tail -20
  exit 1
fi

start=$(date +%s)
docker stop -t 5 "$NAME" >/dev/null
elapsed=$(( $(date +%s) - start ))

if [ "$elapsed" -ge 5 ]; then
  echo "FAIL: did not exit on SIGTERM within the grace period (${elapsed}s)"
  echo "      A container that ignores SIGTERM makes every 'sheppy woof' a stall."
  exit 1
fi

echo "PASS: matched '$PATTERN' in logs and exited on SIGTERM in ${elapsed}s"
