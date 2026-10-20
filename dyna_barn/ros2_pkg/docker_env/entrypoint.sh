#!/usr/bin/env bash
set -e

# --- ROS 2 ---
if [ -f "/opt/ros/humble/setup.bash" ]; then
  . /opt/ros/humble/setup.bash
fi

# Ensure /ws exists & is writable (handles bind-mount perms)
if [ ! -d /ws ]; then
  sudo mkdir -p /ws
fi
sudo chown -R "$(id -u)":"$(id -g)" /ws 2>/dev/null || true
mkdir -p /ws/src

# --- Gazebo Classic defaults (shaders, media, models, etc.) ---
if [ -f "/usr/share/gazebo/setup.sh" ]; then
  . /usr/share/gazebo/setup.sh
fi


# Safety nets in case setup.sh wasn’t present or didn’t populate these:
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}"

# --- Project-specific paths (DynaBARN) ---
# Only append if those dirs exist, and preserve existing paths.
if [ -d "/ws/src/DynaBARN/plugins/build" ]; then
  export GAZEBO_PLUGIN_PATH="/ws/src/DynaBARN/plugins/build:${GAZEBO_PLUGIN_PATH}"
  # Helpful for runtime linker too:
  export LD_LIBRARY_PATH="/ws/src/DynaBARN/plugins/build:${LD_LIBRARY_PATH}"
fi

if [ -d "/ws/src/DynaBARN/worlds" ]; then
  export GAZEBO_RESOURCE_PATH="/ws/src/DynaBARN/worlds:${GAZEBO_RESOURCE_PATH}"
  # Classic models can also live under worlds; include if useful:
  export GAZEBO_MODEL_PATH="/ws/src/DynaBARN/worlds:${GAZEBO_MODEL_PATH}"
fi

# Add TurtleBot3 meshes
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"



# (Optional) Silence audio warnings in containers
export GAZEBO_AUDIO=${GAZEBO_AUDIO:-0}

exec "$@"
