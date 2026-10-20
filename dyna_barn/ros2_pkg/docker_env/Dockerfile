# ------------------------------------------------------------
# ROS 2 Humble (Jammy) + ros_gz + classic Gazebo dev image
# ------------------------------------------------------------
    FROM ros:humble-ros-base-jammy

    ENV DEBIAN_FRONTEND=noninteractive
    
    # Base tools
    RUN apt-get update && apt-get install -y \
        sudo \
        bash-completion \
        build-essential cmake pkg-config \
        git curl wget vim nano \
        python3-pip python3-colcon-common-extensions python3-rosdep \
        # GUI/headless helpers (optional)
        xvfb mesa-utils libgl1-mesa-dri libglu1-mesa \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-turtlebot3-description ros-humble-xacro \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-description \
        ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
        # useful CLI utils
        unzip less \
     && rm -rf /var/lib/apt/lists/*
    
    # rosdep init/update as root
    RUN rosdep init || true && rosdep update
    
    # ros_gz bridge + Gazebo (gz-sim) meta
    RUN apt-get update && apt-get install -y \
        ros-humble-ros-gz \
     && rm -rf /var/lib/apt/lists/*
    
    # Install Gazebo Classic via script
    RUN curl -sSL http://get.gazebosim.org | sh
    
    # Python deps
    RUN pip3 install --no-cache-dir \
        numpy \
        empy \
        setuptools
    
    # Create a non-root user
    ARG USERNAME=dev
    ARG USER_UID=1000
    ARG USER_GID=1000
    RUN groupadd --gid ${USER_GID} ${USERNAME} \
     && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
     && usermod -aG sudo ${USERNAME} \
     && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    
    # Workspace
    ENV WORKSPACE=/ws
    RUN mkdir -p ${WORKSPACE}/src && \
        chown -R ${USER_UID}:${USER_GID} ${WORKSPACE}
    WORKDIR ${WORKSPACE}
    
    # ROS + Gazebo env
    ENV GZ_SIM_RESOURCE_PATH=${WORKSPACE}/install/share:${GZ_SIM_RESOURCE_PATH}
    ENV GZ_SIM_PLUGIN_PATH=${WORKSPACE}/install/lib:${GZ_SIM_PLUGIN_PATH}
    ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${WORKSPACE}/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
    ENV GAZEBO_MODEL_PATH=/usr/share/gazebo:${GAZEBO_MODEL_PATH}
    ENV QT_X11_NO_MITSHM=1
    
    # Entrypoint
    COPY entrypoint.sh /entrypoint.sh
    RUN chmod +x /entrypoint.sh
    
    # Switch to non-root
    USER ${USERNAME}
    
    ENTRYPOINT ["/entrypoint.sh"]
    CMD ["bash"]
    