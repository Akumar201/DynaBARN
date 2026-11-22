#!/usr/bin/env bash
# One training actor: runs Gazebo episodes back to back with the newest policy and saves each
# episode to <run>/buffer for the learner. Runs INSIDE the ROS 2 container (Docker or Apptainer),
# with the built workspace sourced. Several actors may share a machine: each gets its own Gazebo
# partition and ROS domain, everything stays on localhost, and each episode's processes are
# killed as one process group, so actors never touch each other's simulations.
#
# Usage: actor_loop.sh <run dir> <actor id> [episodes, default unlimited]
# Environment:
#   TRAIN_WORLDS   world prefix under share/dynabarn2/worlds (default dynabarn2_train/world_)
#   TRAIN_COUNT    number of training worlds (default: counted from the folder)
#   TRAIN_MODES    obstacle modes to sample from (default "off yield block")
#   EPISODE_TIME   time limit per training episode in seconds (default 60)
# Stops when <run>/STOP exists.
set -u
RUN=$1; ID=$2; EPISODES=${3:-0}
TRAIN_WORLDS=${TRAIN_WORLDS:-dynabarn2_train/world_}
TRAIN_MODES=${TRAIN_MODES:-off yield block}
EPISODE_TIME=${EPISODE_TIME:-60}
SHARE=$(ros2 pkg prefix dynabarn2)/share/dynabarn2/worlds
TRAIN_COUNT=${TRAIN_COUNT:-$(ls "$SHARE/$(dirname "$TRAIN_WORLDS")"/world_*.sdf | wc -l)}
read -r -a MODES <<< "$TRAIN_MODES"
mkdir -p "$RUN/buffer" "$RUN/logs" "$RUN/trials"

export GZ_PARTITION="actor_$(hostname)_$ID" GZ_IP=127.0.0.1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(( (ID % 130) + 101 ))} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export LEARNED_MODE=train LEARNED_EPISODE_DIR="$RUN/buffer" LEARNED_POLICY="$RUN/policy/policy_latest.npz"
LOG="$RUN/logs/actor_$(hostname)_$ID.log"
# ROS and Gazebo write logs for every launch; keep them in node-local scratch, emptied each episode.
export ROS_LOG_DIR=${ROS_LOG_DIR:-${TMPDIR:-/tmp}/dynabarn2_roslog_$GZ_PARTITION}

n=0
while [ ! -e "$RUN/STOP" ]; do
  [ "$EPISODES" -gt 0 ] && [ "$n" -ge "$EPISODES" ] && break
  n=$((n + 1))
  world=$((RANDOM % TRAIN_COUNT)); mode=${MODES[$((RANDOM % ${#MODES[@]}))]}
  echo "$(date +%T) episode $n: ${TRAIN_WORLDS}$world $mode" >> "$LOG"
  setsid timeout $((EPISODE_TIME + 120)) ros2 launch dynabarn2 run_trial.launch.py \
    world_file:="${TRAIN_WORLDS}$world" planner:=learned reactive:="$mode" sync_start:=true \
    timeout:="$EPISODE_TIME" out_file:="$RUN/trials/actor_$(hostname)_$ID.csv" \
    > "$RUN/logs/last_episode_$(hostname)_$ID.log" 2>&1 &
  pid=$!
  wait $pid
  kill -9 -- -$pid 2>/dev/null   # anything left of this episode (Gazebo server, bridges)
  tail -n 1 "$RUN/trials/actor_$(hostname)_$ID.csv" >> "$LOG" 2>/dev/null
  rm -rf "${ROS_LOG_DIR:?}"/*
  # Each launch writes a robot setup folder to /tmp; remove this user's stale ones.
  find /tmp -maxdepth 1 -name 'dynabarn2_robot_*' -user "$(id -u)" -mmin +15 -exec rm -rf {} + 2>/dev/null
done
echo "$(date +%T) actor $ID stopped after $n episodes" >> "$LOG"
