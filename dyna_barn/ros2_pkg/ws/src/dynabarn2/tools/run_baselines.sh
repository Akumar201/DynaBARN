#!/usr/bin/env bash
# Run Nav2 baselines over a set of worlds, each trial in its own isolated container.
# Usage: run_baselines.sh <tag> <first> <last> [world_prefix] [repeats] [parallel] [reactive]
#   e.g. run_baselines.sh dynabarn 0 59 dynabarn/world_ 1 4
# Environment: PLANNERS (default "mppi dwb"), REACTIVE_SPEED (1.0), REACTIVE_RADIUS (2.5), SYNC_START (false),
# CRITIC_PARAMS (JSON overrides for the PredictionCritic, default none),
# LEARNED_POLICY (exported actor .npz for planner "learned"; its folder is mounted read-only).
# Trial CSVs land in dynabarn2/results/<tag>/ and are merged into results/<tag>.csv.
set -u
TAG=$1; FIRST=$2; LAST=$3; PREFIX=${4:-dynabarn/world_}; REPEATS=${5:-1}; PARALLEL=${6:-4}
REACTIVE=${7:-off}    # off | yield | block
PLANNERS=${PLANNERS:-mppi dwb}; REACTIVE_SPEED=${REACTIVE_SPEED:-1.0}; REACTIVE_RADIUS=${REACTIVE_RADIUS:-2.5}; SYNC_START=${SYNC_START:-false}; CRITIC_PARAMS=${CRITIC_PARAMS:-}
LEARNED_POLICY=${LEARNED_POLICY:-}; POLICY_DIR=/nonexistent
[ -n "$LEARNED_POLICY" ] && POLICY_DIR=$(cd "$(dirname "$LEARNED_POLICY")" && pwd) && LEARNED_POLICY=/policy/$(basename "$LEARNED_POLICY")
ROS2_PKG=$(cd "$(dirname "$0")/../../../.." && pwd)   # dyna_barn/ros2_pkg
RES=results/$TAG; mkdir -p "$ROS2_PKG/ws/src/dynabarn2/$RES"

trial() {  # world planner repeat slot
  local name="$1_$2_$3" dir="$ROS2_PKG/ws/src/dynabarn2/$RES"
  [ -s "$dir/$name.csv" ] && return 0   # resume support
  # A no_start (robot never moved in 60 s) is a start-up glitch, not a planner outcome: keep its
  # launch log as <name>.nostart.log and retry once.
  for attempt in 1 2; do
    timeout 300 docker run --rm --network none -e GZ_PARTITION="t$$_$4" -e ROS_DOMAIN_ID=$((10 + $4)) \
      -e CRITIC_PARAMS="$CRITIC_PARAMS" -e LEARNED_POLICY="$LEARNED_POLICY" \
      $([ -d "$POLICY_DIR" ] && echo "-v $POLICY_DIR:/policy:ro") \
      --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -v "$ROS2_PKG/ws:/ws" ros2-jazzy-gz:latest bash -c "
      source /ws/install/setup.bash
      timeout 280 ros2 launch dynabarn2 run_trial.launch.py world_file:=$PREFIX$1 planner:=$2 reactive:=$REACTIVE reactive_speed:=$REACTIVE_SPEED reactive_radius:=$REACTIVE_RADIUS sync_start:=$SYNC_START out_file:=/ws/src/dynabarn2/$RES/$name.csv > /ws/src/dynabarn2/$RES/$name.log 2>&1
      pkill -9 -f 'gz sim' ; true"
    if [ "$attempt" = 1 ] && grep -q ',no_start,' "$dir/$name.csv" 2>/dev/null; then
      mv "$dir/$name.csv" "$dir/$name.nostart"; mv "$dir/$name.log" "$dir/$name.nostart.log"
      continue
    fi
    rm -f "$dir/$name.log"; break
  done
}
export -f trial; export CRITIC_PARAMS LEARNED_POLICY POLICY_DIR ROS2_PKG RES PREFIX REACTIVE REACTIVE_SPEED REACTIVE_RADIUS SYNC_START

for w in $(seq "$FIRST" "$LAST"); do for p in $PLANNERS; do for r in $(seq 1 "$REPEATS"); do
  echo "$w $p $r"; done; done; done |
  xargs -P "$PARALLEL" -L 1 bash -c 'trial $0 $1 $2 $(( ($$ % 40) + 1 ))'

cd "$ROS2_PKG/ws/src/dynabarn2/$RES"
{ head -qn1 ./*.csv | head -1; tail -qn +2 ./*.csv; } > "../$TAG.csv"
echo "merged: $(($(wc -l < "../$TAG.csv") - 1)) trials"
