#!/bin/bash
# Training actors: each array task takes one node and runs ACTORS_PER_NODE Gazebo actors on it.
# Submit (see README.md):  sbatch --export=ALL,RUN=/scratch/$USER/dynabarn2_rl/run1,SIF=$PWD/dynabarn2.sif actors.slurm
# Partition names differ between clusters: check `sinfo` and change --partition if needed.
#SBATCH --job-name=dynabarn2-actors
#SBATCH --partition=normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --mem=48G
#SBATCH --time=1-00:00:00
#SBATCH --array=0-3
#SBATCH --output=dynabarn2-actors_%A_%a.out
set -u
: "${RUN:?set RUN to the shared run directory}" "${SIF:?set SIF to dynabarn2.sif}"
CPUS=${SLURM_CPUS_PER_TASK:-32}
ACTORS=${ACTORS_PER_NODE:-$((CPUS / 4))}      # one actor per 4 cores (measured, see README.md)
TASK=${SLURM_ARRAY_TASK_ID:-0}
LOCAL=${TMPDIR:-/tmp}/dynabarn2_${USER}_${SLURM_JOB_ID:-0}_$TASK   # node-local scratch
mkdir -p "$RUN" "$LOCAL"
echo "$(date) node $(hostname): $ACTORS actors, run $RUN"

for k in $(seq 0 $((ACTORS - 1))); do
  id=$((TASK * 1000 + k))
  mkdir -p "$LOCAL/home_$k"
  # --cleanenv keeps the host's Python/conda settings out of ROS. Each actor gets its own home
  # (Gazebo writes there) and ROS domain; domains are unique within this node.
  # On GPU nodes, USE_GPU=1 draws the lidar on the GPU (--nv), which needs far fewer cores.
  apptainer exec --cleanenv ${USE_GPU:+--nv} --home "$LOCAL/home_$k" --bind "$RUN" \
    --env ROS_DOMAIN_ID=$((101 + (TASK * ACTORS + k) % 130)),TMPDIR="$LOCAL/home_$k" \
    --env TRAIN_MODES="${TRAIN_MODES:-off yield block}",EPISODE_TIME="${EPISODE_TIME:-60}",FRAME_STRIDE="${FRAME_STRIDE:-1}" \
    "$SIF" bash -c "source /opt/dynabarn2/install/setup.bash && \
      /opt/dynabarn2/src/dynabarn2/tools/learned/actor_loop.sh '$RUN' $id" &
  sleep 3   # stagger start-up
done
wait
rm -rf "$LOCAL"
echo "$(date) actors finished"
