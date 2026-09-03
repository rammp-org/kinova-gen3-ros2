# kinova_gen3_ros2 — ROS2 Humble node container (arm64 / Jetson AGX Orin).
#
# Build from the REPO ROOT:
#   docker build -f docker/Dockerfile -t kinova-gen3-ros2:humble .
# or just `make build`. See the Makefile for the run flags (host net + RT caps).
#
# Follows the RAMMP-Software module conventions (ros:humble + Cyclone DDS + an
# entrypoint that sources ROS and the workspace) but stays standalone: it does
# NOT depend on a locally-built rammp-base image. To adopt that base once it is
# published to a registry, change the FROM and drop the apt/entrypoint blocks.
#
# Layer order is deliberate: the slow, rarely-changing steps (apt, pip, vcs
# import, rosdep) come first, so editing node source only re-runs colcon.

FROM ros:humble

SHELL ["/bin/bash", "-c"]
WORKDIR /ros2_ws

# --- system deps --------------------------------------------------------------
# Cyclone DDS matches the rest of the fleet; vcstool pulls the core driver.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        python3-pip \
        python3-vcstool && \
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
COPY kinova_gen3_interfaces/package.xml src/kinova_gen3_ros2/kinova_gen3_interfaces/package.xml
COPY kinova_gen3_ros2/package.xml       src/kinova_gen3_ros2/kinova_gen3_ros2/package.xml
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep install --ignore-src -y --skip-keys pinocchio --from-paths \
      src/kinova_gen3_ros2 \
      src/kinova-gen3-driver \
      src/RAMMP-CuRobo/rammp_curobo_interfaces && \
    rm -rf /var/lib/apt/lists/*

# --- KORTEX SDK (real-arm builds only) ----------------------------------------
# docker/vendor/ is normally empty (just .gitkeep), so this is a no-op layer for
# sim images. `make stage-kortex` rsyncs the proprietary aarch64 SDK into it for
# a real-arm build. It has to be present at BUILD time — libKortexApiCpp.a is
# linked statically, so it cannot be bind-mounted at run time.
COPY docker/vendor/ /opt/kortex/

# --- workspace source + build -------------------------------------------------
COPY . src/kinova_gen3_ros2/

# --packages-up-to kinova_gen3_ros2 builds exactly the node and its recursive
# deps (kinova_lowlevel, kinova_gen3_interfaces, rammp_curobo_interfaces) and
# stops there — rammp_curobo_ros is the GPU planner node and belongs in the
# rammp-curobo image, not this one.
ARG KINOVA_ENABLE_KORTEX=OFF
ARG KORTEX_SDK_DIR=kortex_api_2.8.0_aarch64
RUN source /opt/ros/humble/setup.bash && \
    export CMAKE_PREFIX_PATH="${CMEEL_PREFIX}:${CMAKE_PREFIX_PATH:-}" && \
    colcon build --symlink-install --event-handlers console_direct+ \
      --packages-up-to kinova_gen3_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
        "-DKINOVA_ENABLE_KORTEX=${KINOVA_ENABLE_KORTEX}" \
        "-DKORTEX_HW_DIR=/opt/kortex/${KORTEX_SDK_DIR}"

ENV URDF=/ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf

# --- entrypoint ---------------------------------------------------------------
# Same contract as rammp-base:humble. Built inline so there is no external
# script file to manage or get CRLF-corrupted.
RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/humble/setup.bash' \
      'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' \
      'exec "$@"' \
      > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

# Exec the node binary DIRECTLY, not via `ros2 run`. `ros2 run` forks the real
# binary as a child of a Python wrapper, so the SIGTERM `docker stop` sends to
# PID 1 would hit the wrapper and never reach the node — it would be SIGKILLed
# 10s later with no safe_shutdown(). Exec'd directly, the node IS PID 1 and its
# SIGTERM handler runs the clean stop path. (Same trap as scripts/abra_e2e_sim.sh.)
CMD ["/ros2_ws/install/kinova_gen3_ros2/lib/kinova_gen3_ros2/kinova_gen3_node", \
     "--sim", "--urdf", "/ros2_ws/src/kinova-gen3-driver/models/gen3_7dof_2f85.urdf"]
