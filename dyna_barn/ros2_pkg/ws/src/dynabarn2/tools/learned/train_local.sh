#!/usr/bin/env bash
# Train the learned planner on one machine with Docker: one GPU learner plus several Gazebo actors,
# each in its own container. Stops by itself after HOURS (touches <run>/STOP).
#
# Usage: train_local.sh <run dir> [actors, default 7] [hours, default 12]
# Needs: the ros2-jazzy-gz image, the workspace built (with worlds/dynabarn2_train), and PyTorch
# installed into PYDEPS (default ~/dynabarn2_rl/pydeps), e.g.
#   docker run --rm -v ~/dynabarn2_rl:/d ros2-jazzy-gz:latest pip3 install --break-system-packages --target /d/pydeps torch
# FRAME_STRIDE=2 spaces the 4 stacked scans 0.2 s apart (default 1: 0.1 s).
# Restarting with the same run dir resumes training (the learner reloads checkpoint and episodes).
# Watch: column -s, -t < <run>/progress.csv | tail      Stop early: touch <run>/STOP
set -u
RUN=$(realpath -m "$1"); ACTORS=${2:-7}; HOURS=${3:-12}
PYDEPS=$(realpath -m "${PYDEPS:-$HOME/dynabarn2_rl/pydeps}")
WS=$(cd "$(dirname "$0")/../../../.." && pwd)            # dyna_barn/ros2_pkg/ws
TAG=dynabarn2_rl_$(basename "$RUN")
mkdir -p "$RUN"; rm -f "$RUN/STOP"
[ -d "$WS/install/dynabarn2/share/dynabarn2/worlds/dynabarn2_train" ] || {
  echo "Training worlds missing: generate worlds/dynabarn2_train (seed 2030, --per_cell 20) and rebuild"; exit 1; }
[ -d "$PYDEPS/torch" ] || { echo "PyTorch not found in $PYDEPS"; exit 1; }
echo "$(date) training $RUN: $ACTORS actors for $HOURS h" | tee -a "$RUN/train_local.log"

docker run -d --rm --name "${TAG}_learner" --gpus all --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e PYTHONPATH=/pydeps -v "$PYDEPS:/pydeps:ro" -v "$WS:/ws:ro" -v "$RUN:/run_dir" ros2-jazzy-gz:latest \
  bash -c "python3 /ws/src/dynabarn2/tools/learned/learner.py --run /run_dir --replay 1000000 --frame_stride ${FRAME_STRIDE:-1} ${LEARNER_ARGS:-} \
           >> /run_dir/learner.log 2>&1" > /dev/null

for k in $(seq 0 $((ACTORS - 1))); do
  docker run -d --rm --name "${TAG}_actor$k" --network none -v "$WS:/ws" -v "$RUN:/run_dir" \
    -e TRAIN_MODES="${TRAIN_MODES:-off yield block}" -e EPISODE_TIME="${EPISODE_TIME:-60}" -e FRAME_STRIDE="${FRAME_STRIDE:-1}" \
    ros2-jazzy-gz:latest bash -c "source /ws/install/setup.bash && \
      /ws/src/dynabarn2/tools/learned/actor_loop.sh /run_dir $k" > /dev/null
  sleep 3
done

end=$(( $(date +%s) + HOURS * 3600 ))
while [ "$(date +%s)" -lt "$end" ] && [ ! -e "$RUN/STOP" ]; do sleep 60; done
touch "$RUN/STOP"
echo "$(date) stop requested; waiting for containers" | tee -a "$RUN/train_local.log"
sleep 240
docker rm -f $(docker ps -q --filter "name=${TAG}_") > /dev/null 2>&1
echo "$(date) done: $(ls "$RUN/buffer" | wc -l) episodes; $(tail -n 1 "$RUN/progress.csv" 2>/dev/null)" \
  | tee -a "$RUN/train_local.log"
