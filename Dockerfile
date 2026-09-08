# kinova_gen3_ros2 — ROS2 Humble node container (arm64 / Jetson AGX Orin).
#
# Build from the REPO ROOT:
#   docker build -t kinova-gen3-ros2:humble .
# or just `make build`. See the Makefile for the run flags (host net + RT caps).
#
# Builds on rammp-base, which supplies ROS 2 Humble, Cyclone DDS (selected, with
# the fleet's config), the shared entrypoint, and the interface contract
# compiled in. So this file installs none of those.
#
# THE BASE TAG IS THE INTERFACE PIN. rammp_arm_interfaces arrives inside
# rammp-base, at whatever ref that image was built with, so "which contract does
# this node speak" is answered by the FROM line -- not by kinova_gen3.repos,
# which deliberately no longer lists the interfaces. Two modules interoperate if
# they share a base tag.
#
# arm64 ONLY, because rammp-base publishes arm64 only. This module built both
# architectures on ros:humble, and amd64 let a laptop drive the arm over IP;
# that returns when rammp-org/RAMMP-docker#10 publishes a multi-arch base.
#
# Layer order is deliberate: the slow, rarely-changing steps (pip, vcs import,
# rosdep) come first, so editing node source only re-runs colcon.

FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

SHELL ["/bin/bash", "-c"]

# /module_ws is an overlay ON TOP of the base's /ros2_ws, which is where the
# interfaces are already built. Building here rather than into /ros2_ws is what
# stops this image recompiling -- or silently shipping a modified copy of -- the
# contract it is supposed to be a consumer of. The base entrypoint sources both.
WORKDIR /module_ws

# vcstool is not in the base: it is a build-time tool for pulling THIS module's
# sources, not part of the runtime contract every module inherits.
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro humble

# --- pinocchio ----------------------------------------------------------------
# The whole cmeel stack is version-locked in docker/requirements.txt to what the
# Jetson runs — see that file for why pinning `pin` alone is not enough.
# Deliberately NOT ros-humble-pinocchio: that is 4.0.0 on Humble arm64, a major
# version ahead of what the core driver is validated against. The pip (cmeel)
# wheels also land at the same prefix the bare-metal build uses, so the CMake
# incantation below is identical to scripts/abra_colcon.sh.
COPY docker/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt
ENV CMEEL_PREFIX=/usr/local/lib/python3.10/dist-packages/cmeel.prefix

# --- core driver source -------------------------------------------------------
# kinova-gen3-driver is public, so this clones anonymously (no SSH key in the
# image). CORE_REF overrides the ref pinned in kinova_gen3.repos.
ARG CORE_REF=
COPY kinova_gen3.repos /tmp/kinova_gen3.repos
RUN mkdir -p src && vcs import src < /tmp/kinova_gen3.repos && \
    if [ -n "${CORE_REF}" ]; then git -C src/kinova-gen3-driver fetch --depth 1 origin "${CORE_REF}" && \
        git -C src/kinova-gen3-driver checkout FETCH_HEAD; fi

# --- ROS deps (cached unless a package.xml changes) ---------------------------
# --skip-keys pinocchio: the core's package.xml declares it, but rosdep would
# resolve it to the apt 4.0.0 package and shadow the pinned wheel above.
#
# The paths are enumerated rather than a bare `src`: kinova_gen3.repos vendors all
# of RAMMP-CuRobo, but this image only ever builds rammp_curobo_interfaces (the
# dependency-free IDL). A bare `src` would also install rammp_curobo_ros's deps
# for a package this image never runs — that node lives in the GPU container.
COPY kinova_gen3_ros2/package.xml       src/kinova_gen3_ros2/kinova_gen3_ros2/package.xml
RUN source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    apt-get update && \
    rosdep install --ignore-src -y --skip-keys pinocchio --from-paths \
      src/kinova_gen3_ros2 \
      src/kinova-gen3-driver \
      src/RAMMP-CuRobo/rammp_curobo_interfaces && \
    rm -rf /var/lib/apt/lists/*

# --- KORTEX SDK (real-arm builds only) ----------------------------------------
# Two ways the SDK arrives, checked in that order:
#
#   1. STAGED. `make stage-kortex` rsyncs it into docker/vendor/ from a local
#      copy. docker/vendor/ is normally empty (just .gitkeep), so this COPY is a
#      no-op layer for sim images. Staging wins when present: it needs no
#      network and it lets you build against a specific local SDK.
#
#   2. FETCHED. Otherwise, a KORTEX build downloads it from Kinova's public
#      artifactory. That URL needs no credentials, which is what lets CI build
#      the real-arm image at all: with no fetch path, the SDK would have to come
#      from a developer's machine and the KORTEX build would have no automated
#      coverage, letting a change that broke only it reach the robot uncaught.
#
# Either way it must be present at BUILD time: libKortexApiCpp.a is linked
# statically, so it cannot be bind-mounted at run time -- and it is why the
# published image embeds the SDK rather than expecting one on the host.
ARG KINOVA_ENABLE_KORTEX=OFF
ARG KORTEX_SDK_DIR=kortex_api_2.8.0_aarch64
# Kinova ships a per-architecture SDK, so the URL follows the build platform.
# TARGETARCH is set automatically by buildx.
#   arm64 -> linux_aarch64_gcc_7.4.zip   (the Jetson; the proven path)
#   amd64 -> linux_x86-64_gcc_5.4.zip    (newest x86-64 Kinova publishes for 2.8.0)
# Note the mismatch: aarch64 gets a gcc 7.4 build, x86-64 only a gcc 5.4 one.
ARG TARGETARCH
ARG KORTEX_SDK_BASE=https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0
COPY docker/vendor/ /opt/kortex/
RUN if [ "${KINOVA_ENABLE_KORTEX}" = "ON" ] && \
       [ ! -d "/opt/kortex/${KORTEX_SDK_DIR}/lib" ]; then \
      case "${TARGETARCH}" in \
        arm64) SDK_ZIP=linux_aarch64_gcc_7.4.zip ;; \
        amd64) SDK_ZIP=linux_x86-64_gcc_5.4.zip ;; \
        *) echo "no KORTEX SDK published for TARGETARCH=${TARGETARCH}" >&2; exit 1 ;; \
      esac && \
      echo "KORTEX SDK not staged; fetching ${KORTEX_SDK_BASE}/${SDK_ZIP}" && \
      curl -fsSL "${KORTEX_SDK_BASE}/${SDK_ZIP}" -o /tmp/kortex.zip && \
      mkdir -p "/opt/kortex/${KORTEX_SDK_DIR}" && \
      python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" \
        /tmp/kortex.zip "/opt/kortex/${KORTEX_SDK_DIR}" && \
      rm -f /tmp/kortex.zip && \
      test -f "/opt/kortex/${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp.a"; \
    fi

# --- workspace source + build -------------------------------------------------
COPY . src/kinova_gen3_ros2/

# --packages-up-to kinova_gen3_ros2 builds exactly the node and its recursive
# deps (kinova_lowlevel, rammp_arm_interfaces, rammp_curobo_interfaces) and
# stops there — rammp_curobo_ros is the GPU planner node and belongs in the
# rammp-curobo image, not this one.
RUN source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    export CMAKE_PREFIX_PATH="${CMEEL_PREFIX}:${CMAKE_PREFIX_PATH:-}" && \
    colcon build --symlink-install --event-handlers console_direct+ \
      --packages-up-to kinova_gen3_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
        "-DKINOVA_ENABLE_KORTEX=${KINOVA_ENABLE_KORTEX}" \
        "-DKORTEX_HW_DIR=/opt/kortex/${KORTEX_SDK_DIR}"

ENV URDF=/module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf

# --- entrypoint ---------------------------------------------------------------
# Supplied by rammp-base: it sources ROS, then /ros2_ws (the contract), then
# /module_ws (this node), then execs the command. Deliberately not overridden --
# a module that writes its own drifts from every other module's startup.
# Exec the node binary DIRECTLY, not via `ros2 run`. `ros2 run` forks the real
# binary as a child of a Python wrapper, so the SIGTERM `docker stop` sends to
# PID 1 would hit the wrapper and never reach the node — it would be SIGKILLed
# 10s later with no safe_shutdown(). Exec'd directly, the node IS PID 1 and its
# SIGTERM handler runs the clean stop path. (Same trap as scripts/abra_e2e_sim.sh.)
CMD ["/module_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node", \
     "--sim", "--urdf", "/module_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf"]
