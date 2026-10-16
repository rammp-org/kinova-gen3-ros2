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
# The node reads its model from /arm/robot_description and exits without it, so
# arm_state_publisher runs first, from the same image, on the level TEST fixture
# bind-mounted where a robot's mount.yaml goes.
MOUNT="$(cd "$(dirname "$0")/.." && pwd)/kinova_gen3_description/test/mounts/level.yaml"

docker run -d --rm --name "$NAME-desc" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v "$MOUNT:/config/robot/mount.yaml:ro" "$IMAGE" \
  ros2 launch kinova_gen3_description description.launch.py \
  mount_file:=/config/robot/mount.yaml >/dev/null
trap 'docker rm -f "$NAME" "$NAME-desc" >/dev/null 2>&1 || true' EXIT

docker run -d --rm --name "$NAME" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" >/dev/null

sleep 10

# Capture, then grep. `docker logs | grep -q` under pipefail is a false FAIL:
# grep -q exits at the first match, docker logs takes SIGPIPE (141) while still
# writing, and pipefail reports the pipeline as failed -- certain when the match
# is near the top of a long log, as the embodiment line is.
desc_logs="$(docker logs "$NAME-desc" 2>&1)"
if ! grep -q "embodiment: test_level" <<<"$desc_logs"; then
  echo "FAIL: arm_state_publisher did not log its embodiment line"
  tail -20 <<<"$desc_logs"
  exit 1
fi

node_logs="$(docker logs "$NAME" 2>&1)"
if ! grep -q "$PATTERN" <<<"$node_logs"; then
  echo "FAIL: no match for pattern '$PATTERN' in the container logs"
  tail -20 <<<"$node_logs"
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
