#!/usr/bin/env bash
# Run Nav2 baselines over a set of worlds, each trial in its own isolated container.
# Usage: run_baselines.sh <tag> <first> <last> [world_prefix] [repeats] [parallel] [reactive]
#   e.g. run_baselines.sh dynabarn 0 59 dynabarn/world_ 1 4
# Trial CSVs land in dynabarn2/results/<tag>/ and are merged into results/<tag>.csv.
set -u
TAG=$1; FIRST=$2; LAST=$3; PREFIX=${4:-dynabarn/world_}; REPEATS=${5:-1}; PARALLEL=${6:-4}
REACTIVE=${7:-off}    # off | yield | block
ROS2_PKG=$(cd "$(dirname "$0")/../../../.." && pwd)   # dyna_barn/ros2_pkg
RES=results/$TAG; mkdir -p "$ROS2_PKG/ws/src/dynabarn2/$RES"

trial() {  # world planner repeat slot
  local out="/ws/src/dynabarn2/$RES/$1_$2_$3.csv"
  [ -s "$ROS2_PKG/ws/src/dynabarn2/$RES/$1_$2_$3.csv" ] && return 0   # resume support
  timeout 300 docker run --rm --network none -e GZ_PARTITION="t$$_$4" -e ROS_DOMAIN_ID=$((10 + $4)) \
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -v "$ROS2_PKG/ws:/ws" ros2-jazzy-gz:latest bash -c "
    source /ws/install/setup.bash
    timeout 280 ros2 launch dynabarn2 run_trial.launch.py world_file:=$PREFIX$1 planner:=$2 reactive:=$REACTIVE out_file:=$out > /dev/null 2>&1
    pkill -9 -f 'gz sim' ; true"
}
export -f trial; export ROS2_PKG RES PREFIX REACTIVE

for w in $(seq "$FIRST" "$LAST"); do for p in mppi dwb; do for r in $(seq 1 "$REPEATS"); do
  echo "$w $p $r"; done; done; done |
  xargs -P "$PARALLEL" -L 1 bash -c 'trial $0 $1 $2 $(( ($$ % 40) + 1 ))'

cd "$ROS2_PKG/ws/src/dynabarn2/$RES"
{ head -qn1 ./*.csv | head -1; tail -qn +2 ./*.csv; } > "../$TAG.csv"
echo "merged: $(($(wc -l < "../$TAG.csv") - 1)) trials"
