#!/bin/bash
# Learner: one GPU. Resumes from <RUN>/checkpoint.pt, so just resubmit if the time limit ends it.
# Submit (see README.md):  sbatch --export=ALL,RUN=/scratch/$USER/dynabarn2_rl/run1,REPO=$HOME/DynaBARN,VENV=$HOME/dynabarn2_venv learner.slurm
# Partition and GPU names differ between clusters: check `sinfo` and change them if needed.
#SBATCH --job-name=dynabarn2-learner
#SBATCH --partition=gpuq
#SBATCH --gres=gpu:1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=2-00:00:00
#SBATCH --output=dynabarn2-learner_%j.out
set -u
: "${RUN:?set RUN}" "${REPO:?set REPO to the DynaBARN git checkout}" "${VENV:?set VENV to the Python venv with torch}"
source "$VENV/bin/activate"
nvidia-smi --query-gpu=name,memory.total --format=csv || true
python "$REPO/dyna_barn/ros2_pkg/ws/src/dynabarn2/tools/learned/learner.py" --run "$RUN" --frame_stride ${FRAME_STRIDE:-1} ${LEARNER_ARGS:-}
