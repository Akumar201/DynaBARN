# ------------------------------------------------------------
# ROS 2 Jazzy (Noble) + Gazebo Harmonic (gz-sim 8) dev image
# ------------------------------------------------------------
    FROM ros:jazzy-ros-base-noble

    ENV DEBIAN_FRONTEND=noninteractive

    # Base tools
    RUN apt-get update && apt-get install -y \
        sudo \
        bash-completion \
        build-essential cmake pkg-config \
        git curl wget vim nano \
        python3-pip python3-colcon-common-extensions python3-rosdep \
        # Python deps (pip is externally managed on Noble, so use apt)
        python3-numpy python3-empy python3-setuptools \
        # GUI/headless helpers (optional)
        xvfb mesa-utils libgl1-mesa-dri libglu1-mesa \
        # useful CLI utils
        unzip less \
     && rm -rf /var/lib/apt/lists/*

    # rosdep init/update as root
    RUN rosdep init || true && rosdep update

    # Gazebo Harmonic (via ROS vendor packages) + ros_gz bridge, plus navigation tooling
    RUN apt-get update && apt-get install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-teleop-twist-keyboard \
        # Clearpath Jackal (J100) description, Gazebo bringup and Nav2 examples
        ros-jazzy-clearpath-simulator \
        ros-jazzy-clearpath-nav2-demos \
     && rm -rf /var/lib/apt/lists/*

    # Create a non-root user (Noble images ship an "ubuntu" user on UID/GID 1000)
    ARG USERNAME=dev
    ARG USER_UID=1000
    ARG USER_GID=1000
    RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
     && groupadd --gid ${USER_GID} ${USERNAME} \
     && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
     && usermod -aG sudo ${USERNAME} \
     && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

    # Workspace
    ENV WORKSPACE=/ws
    RUN mkdir -p ${WORKSPACE}/src && \
        chown -R ${USER_UID}:${USER_GID} ${WORKSPACE}
    WORKDIR ${WORKSPACE}

    ENV QT_X11_NO_MITSHM=1

    # Entrypoint
    COPY entrypoint.sh /entrypoint.sh
    RUN chmod +x /entrypoint.sh

    # Switch to non-root
    USER ${USERNAME}

    ENTRYPOINT ["/entrypoint.sh"]
    CMD ["bash"]
