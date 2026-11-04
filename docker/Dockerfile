# ROS 2 + Gazebo image for dynabarn-core.
#
#   docker compose -f docker/compose.yaml build              # Jazzy + Harmonic (default)
#
# GPU-agnostic: nothing here depends on a GPU vendor. Rendering uses whatever
# OpenGL the container is given (see compose.yaml): Mesa software rendering by
# default, /dev/dri for Intel/AMD, or the NVIDIA runtime via an override file.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

# ros_gz pulls in the Gazebo release paired with this ROS distro (Harmonic on Jazzy).
# Mesa provides OpenGL/EGL for software rendering and for Intel/AMD GPUs; with
# the NVIDIA runtime, libglvnd dispatches to the host driver instead.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-launch-pytest \
        python3-colcon-common-extensions \
        python3-venv python3-pip \
        libgl1 libegl1 libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libosmesa6 \
        libglfw3 mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host user, so files written to the mounted repo
# stay yours. The base image already has uid 1000 ("ubuntu") on 24.04.
RUN if getent passwd ${USER_UID} >/dev/null; then \
        usermod -l dev -d /home/dev -m "$(getent passwd ${USER_UID} | cut -d: -f1)"; \
    else \
        groupadd -g ${USER_GID} dev 2>/dev/null || true; \
        useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash dev; \
    fi \
    && usermod -aG video dev && (getent group render && usermod -aG render dev || true)

# Python env that also sees the ROS packages (rclpy etc.); dynabarn-core's own
# dependencies come from pip so they match what pip users get.
RUN python3 -m venv --system-site-packages /opt/venv && chown -R ${USER_UID}:${USER_GID} /opt/venv
ENV PATH=/opt/venv/bin:$PATH \
    VIRTUAL_ENV=/opt/venv
RUN pip install --no-cache-dir "mujoco>=3.0" pyyaml pytest

# colcon workspace lives outside the repo so build/ install/ log/ never land in it
RUN mkdir -p /ws && chown ${USER_UID}:${USER_GID} /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dev
WORKDIR /work
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
