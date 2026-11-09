#!/usr/bin/env bash
# Live demo: two planners on the same world, side by side, each in its own Gazebo window.
# Usage: demo.sh [world] [reactive] [left planner] [right planner]
#   default: demo.sh dynabarn2/world_13 block mppi mppi_critic
# Good picks (critic succeeds 3/3, plain MPPI collides 3/3):
#   block: dynabarn2/world_13 16 19 26 31 38 48     fixed: world_0 3 29 38 48     yield: world_3 12 29
# SYNC_START=false shows the old timing bug: obstacles start before the robot, so runs differ.
# Each trial prints its result line (status, time, score) when it ends. Ctrl-C stops both.
set -u
SYNC=${SYNC_START:-true}; WORLD=${1:-dynabarn2/world_13}; REACTIVE=${2:-block}; LEFT=${3:-mppi}; RIGHT=${4:-mppi_critic}
ROS2_PKG=$(cd "$(dirname "$0")/../../../.." && pwd)
OUT=/ws/src/dynabarn2/results/demo; mkdir -p "$ROS2_PKG/ws/src/dynabarn2/results/demo"
xhost +local: > /dev/null

run() {  # planner slot
  docker run --rm --name "dynabarn_demo_$2" --network none -e GZ_PARTITION="demo_$2" \
    -e ROS_DOMAIN_ID=$((90 + $2)) --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$ROS2_PKG/ws:/ws" ros2-jazzy-gz:latest bash -c "
    source /ws/install/setup.bash
    f=$OUT/\$(echo $WORLD | tr / _)_${REACTIVE}_$1.csv; rm -f \$f
    ros2 launch dynabarn2 run_trial.launch.py world_file:=$WORLD planner:=$1 reactive:=$REACTIVE \
      sync_start:=$SYNC gui:=true out_file:=\$f > /dev/null 2>&1
    echo \"[$1] \$(tail -1 \$f)\""
}
trap 'docker kill dynabarn_demo_1 dynabarn_demo_2 > /dev/null 2>&1' INT TERM
echo "World $WORLD, obstacles $REACTIVE: left window $LEFT, right window $RIGHT (opens second), sync_start=$SYNC"
run "$LEFT" 1 & sleep 5; run "$RIGHT" 2 & wait   # right window opens second; drag them apart
