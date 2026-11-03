#!/usr/bin/env bash
set -e

# --- ROS 2 ---
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  . /opt/ros/jazzy/setup.bash
fi

# Ensure /ws exists & is writable (handles bind-mount perms)
if [ ! -d /ws ]; then
  sudo mkdir -p /ws
fi
sudo chown -R "$(id -u)":"$(id -g)" /ws 2>/dev/null || true
mkdir -p /ws/src

# --- Workspace overlay (if built) ---
if [ -f "/ws/install/setup.bash" ]; then
  . /ws/install/setup.bash
fi

# --- Gazebo Harmonic paths ---
# Worlds/models from the workspace, and system plugins (e.g. the DynaBARN obstacle mover)
export GZ_SIM_RESOURCE_PATH="/ws/install/share:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/ws/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

if [ -d "/ws/src/DynaBARN/worlds" ]; then
  export GZ_SIM_RESOURCE_PATH="/ws/src/DynaBARN/worlds:${GZ_SIM_RESOURCE_PATH}"
fi

exec "$@"
