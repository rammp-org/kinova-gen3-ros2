# Container workflow for kinova_arm_ros2. Run these ON the Jetson (arm64).
#
#   make build          build the sim image
#   make sim            run the node in sim (foreground)
#   make e2e            run the two-goal sim integration check
#   make shell          interactive shell in the image
#   make stage-kortex   copy the aarch64 KORTEX SDK into the build context
#   make real IP=...    build KORTEX-enabled and run against the arm (attended)
#
# Plain docker — no compose dependency. The point of these targets is that the
# run flags are not something to retype from memory: this node needs host
# networking for DDS and RT privileges for its SCHED_FIFO loop, and a container
# missing either fails in a confusing way rather than an obvious one.

IMAGE       ?= kinova-arm-ros2:humble
IMAGE_REAL  ?= kinova-arm-ros2:kortex
# Override the core ref that kinova_arm.repos pins, e.g.
#   make build CORE_REF=feat/planning-failed-result-code
# Needed whenever this repo depends on a core change that has not reached core
# main yet — the container clones main, unlike the rsync dev loop.
CORE_REF    ?=
CORE_ARG    := $(if $(CORE_REF),--build-arg CORE_REF=$(CORE_REF),)
KORTEX_SRC  ?= $(HOME)/kortex_api_2.8.0_aarch64
KORTEX_SDK_DIR := $(notdir $(KORTEX_SRC))

NODE   := /ros2_ws/install/kinova_arm_ros2/lib/kinova_arm_ros2/kinova_arm_node
CLIENT := /ros2_ws/src/kinova_arm_ros2/kinova_arm_ros2/test/send_trajectory.py
URDF   := /ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf

# Pin the RT loop to abra's isolated core. The host boots with
# `isolcpus=11 nohz_full=11 rcu_nocbs=11` and the core driver's scripts/rt_setup.sh
# defaults RT_CORE=11. isolcpus takes that core OUT of the scheduler's load
# balancing, so a thread only ever lands there via explicit affinity — without
# --cpu the node calls no sched_setaffinity at all (enable_rt guards it on
# cpu >= 0) and the 1 kHz loop runs on the general cores forever.
#
# --cpu pins ONLY the RT loop: enable_rt() runs inside RtExecutor::run() on the
# main thread, after bringup_node has already spawned the rclcpp spin and
# telemetry-drain threads, so those keep the full mask. Do NOT reach for docker's
# --cpuset-cpus instead: that confines the whole container, ROS threads included,
# to the isolated core — the opposite of what the isolation is for.
RT_CORE ?= 11
NODE_ARGS := --urdf $(URDF) --cpu $(RT_CORE)

# DDS needs the host net + a shared IPC namespace (shared-memory transport).
ROS_FLAGS := --network host --ipc host -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# mlockall + SCHED_FIFO(80) + affinity, per bringup_node. SYS_NICE is what lets
# sched_setscheduler succeed; the ulimits raise the in-container ceilings.
# Full --privileged is NOT needed.
RT_FLAGS  := --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1
RUN       := docker run --rm $(ROS_FLAGS) $(RT_FLAGS)

.PHONY: build build-real sim e2e shell stage-kortex real

build:                     ## Build the sim image
	docker build -f docker/Dockerfile $(CORE_ARG) -t $(IMAGE) .

sim: build                 ## Run the node in sim, foreground
	$(RUN) -it --name kinova_arm_sim $(IMAGE) $(NODE) --sim $(NODE_ARGS)

# Success case then forced-divergence case, same assertions as
# scripts/abra_e2e_sim.sh but against the containerized node.
e2e: build                 ## Sim integration check (success + path-tolerance abort)
	$(RUN) -d --name kinova_arm_e2e $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	@sleep 5
	@set -e; trap 'docker rm -f kinova_arm_e2e >/dev/null' EXIT; \
	  docker exec kinova_arm_e2e /ros_entrypoint.sh python3 $(CLIENT) \
	    --mode position --delta 0.05 --dur 0.4 --expect 0; \
	  docker exec kinova_arm_e2e /ros_entrypoint.sh python3 $(CLIENT) \
	    --delta 0.5 --dur 2.0 --path-tol 0.2 --expect -4; \
	  echo "--- node scheduling policy (expect SCHED_FIFO/80) ---"; \
	  docker exec kinova_arm_e2e chrt -p 1

shell:                     ## Interactive shell in the image
	$(RUN) -it $(IMAGE) bash

# The KORTEX SDK is proprietary and NOT in git. Docker can only read the build
# context, so it has to be copied in (not symlinked) before a real-arm build.
stage-kortex:              ## Copy the aarch64 KORTEX SDK into docker/vendor/
	@test -d "$(KORTEX_SRC)" || { echo "no KORTEX SDK at $(KORTEX_SRC); set KORTEX_SRC=..."; exit 1; }
	rsync -a --delete "$(KORTEX_SRC)/" "docker/vendor/$(KORTEX_SDK_DIR)/"
	@echo "staged $(KORTEX_SDK_DIR) -> docker/vendor/"

# Build the real-arm image WITHOUT running it — safe off-robot, and the step you
# want ahead of an attended session so the ~minutes of build are not happening
# with the arm powered and someone holding the e-stop.
build-real: stage-kortex   ## Build the KORTEX-enabled image only
	docker build -f docker/Dockerfile $(CORE_ARG) --build-arg KINOVA_ENABLE_KORTEX=ON \
	  --build-arg KORTEX_SDK_DIR=$(KORTEX_SDK_DIR) -t $(IMAGE_REAL) .

# ATTENDED ONLY — docs/on-robot-runbook.md. e-stop in hand.
real: build-real           ## Build KORTEX-enabled and run against the arm
	@test -n "$(IP)" || { echo "usage: make real IP=192.168.1.10"; exit 1; }
	$(RUN) -it --name kinova_arm_real --stop-timeout 20 $(IMAGE_REAL) \
	  $(NODE) --ip $(IP) $(NODE_ARGS)
