# Container workflow for kinova_gen3_ros2. Run these ON the Jetson (arm64).
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

IMAGE       ?= kinova-gen3-ros2:humble
IMAGE_REAL  ?= kinova-gen3-ros2:kortex
# Override the core ref that kinova_gen3.repos pins, e.g.
#   make build CORE_REF=feat/planning-failed-result-code
# Needed whenever this repo depends on a core change that has not reached core
# main yet — the container clones the pinned ref, not your local checkout.
CORE_REF    ?=
CORE_ARG    := $(if $(CORE_REF),--build-arg CORE_REF=$(CORE_REF),)
KORTEX_SRC  ?= $(HOME)/kortex_api_2.8.0_aarch64
KORTEX_SDK_DIR := $(notdir $(KORTEX_SRC))

# /module_ws, not /ros2_ws. The image builds FROM rammp-base, so /ros2_ws is the
# BASE's workspace -- the interface contract, already built -- and this node
# overlays into /module_ws (Dockerfile WORKDIR, and `COPY . src/kinova_gen3_ros2/`
# is relative to it). The old paths built fine and then died at `docker run` on a
# path the image does not contain.
NODE   := /module_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node
CLIENT := /module_ws/src/kinova_gen3_ros2/kinova_gen3_ros2/test/send_trajectory.py
URDF   := /module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf

# Pin the RT loop to the host's isolated core. Boot the host with e.g.
# `isolcpus=11 nohz_full=11 rcu_nocbs=11` (the core driver's scripts/rt_setup.sh
# defaults RT_CORE=11), or pass RT_CORE=-1 on a machine with no isolated core.
# isolcpus takes that core OUT of the scheduler's load
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

.PHONY: build build-real sim e2e shell stage-kortex real run check smoke lint

build:                     ## Build the sim image
	docker build $(CORE_ARG) -t $(IMAGE) .

sim: build                 ## Run the node in sim, foreground
	$(RUN) -it --name kinova_gen3_sim $(IMAGE) $(NODE) --sim $(NODE_ARGS)

# Success case then forced-divergence case, same assertions as the README's
# native sim end-to-end, but against the containerized node.
e2e: build                 ## Sim integration check (success + path-tolerance abort)
	$(RUN) -d --name kinova_gen3_e2e $(IMAGE) $(NODE) --sim $(NODE_ARGS)
	@sleep 5
	@set -e; trap 'docker rm -f kinova_gen3_e2e >/dev/null' EXIT; \
	  docker exec kinova_gen3_e2e /ros_entrypoint.sh python3 $(CLIENT) \
	    --mode position --delta 0.05 --dur 0.4 --expect 0; \
	  docker exec kinova_gen3_e2e /ros_entrypoint.sh python3 $(CLIENT) \
	    --delta 0.5 --dur 2.0 --path-tol 0.2 --expect -4; \
	  echo "--- node scheduling policy (expect SCHED_FIFO/80) ---"; \
	  docker exec kinova_gen3_e2e chrt -p 1

# --- the RAMMP module standard's targets -------------------------------------
# These four are what `rammp-module-template` expects every module to answer to.
# The targets above stay: they carry the RT run flags and the KORTEX staging
# step, which the standard has no place for and which are the difference
# between a working arm and a confusing failure.

run:                       ## Run the sim image, the way the standard does
	$(RUN) -it $(IMAGE) $(NODE) --sim $(NODE_ARGS)

# Validated against sheppy itself rather than a copied script -- see
# scripts/check_fragments.py for why.
check:                     ## Validate the fragments
	uv run --with pyyaml --with "sheppy @ git+https://github.com/rammp-org/sheppy@main" \
	  python3 scripts/check_fragments.py rammp-alternative*.yaml

smoke:                     ## Does it publish, and does it exit on SIGTERM?
	SMOKE_PATTERN="kinova_gen3_node up" ./scripts/smoke.sh $(IMAGE)

lint:                      ## pre-commit run --all-files
	pre-commit run --all-files

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
	docker build $(CORE_ARG) --build-arg KINOVA_ENABLE_KORTEX=ON \
	  --build-arg KORTEX_SDK_DIR=$(KORTEX_SDK_DIR) -t $(IMAGE_REAL) .

# ATTENDED ONLY — docs/on-robot-runbook.md. e-stop in hand.
real: build-real           ## Build KORTEX-enabled and run against the arm
	@test -n "$(IP)" || { echo "usage: make real IP=192.168.1.10"; exit 1; }
	$(RUN) -it --name kinova_gen3_real --stop-timeout 20 $(IMAGE_REAL) \
	  $(NODE) --ip $(IP) $(NODE_ARGS)
