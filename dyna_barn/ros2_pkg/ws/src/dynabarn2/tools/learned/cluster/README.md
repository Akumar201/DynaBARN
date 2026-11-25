# Training the DynaBARN II learned planner on a Slurm cluster

To train on one machine with Docker instead, run `tools/learned/train_local.sh <run dir> [actors] [hours]`.

This trains a navigation policy with reinforcement learning (Soft Actor-Critic). The policy
looks at the lidar and the robot's odometry and outputs speed and turn rate. It is then
compared with MPPI, RPP and our prediction planner on the test worlds.

There are two kinds of job, and both share one run folder:

- **Actors** (CPU nodes). Each actor runs Gazebo episodes, one after another, on training worlds
  (`worlds/dynabarn2_train`, 300 worlds, generator seed 2030). The training worlds never include
  the test worlds. They are not in git: `build_image.sh` regenerates them exactly from the seed. Each actor drives with the newest policy and saves every episode to
  `<run>/buffer/`.
- **Learner** (one GPU). It reads new episodes, trains, and keeps writing the newest policy to
  `<run>/policy/policy_latest.npz`.

Until the learner has collected 20,000 steps (about 80 episodes), actors drive with a simple
"head for the goal" rule. Training starts after that.

## 1. One-time setup

**Image.** Someone with Docker builds the image on their own machine and copies it to the
cluster:

```bash
dyna_barn/ros2_pkg/ws/src/dynabarn2/tools/learned/cluster/build_image.sh dynabarn2_image.tar
scp dynabarn2_image.tar <cluster>:/scratch/$USER/
```

Then on the cluster:

```bash
cd /scratch/$USER
module load apptainer 2>/dev/null || module load singularity   # the module name may differ
apptainer build dynabarn2.sif dynabarn2_image.tar               # older versions: docker-archive://dynabarn2_image.tar
```

**Code and Python for the learner.**

```bash
git clone -b prarthana/dynabarn2-jazzy-harmonic <repo url> $HOME/DynaBARN
python3 -m venv $HOME/dynabarn2_venv
$HOME/dynabarn2_venv/bin/pip install torch numpy        # the default torch wheel includes CUDA
```

**Quick check, about 2 minutes.** On any node, run one actor for one episode:

```bash
export RUN=/scratch/$USER/dynabarn2_rl/check
apptainer exec --cleanenv --bind /scratch dynabarn2.sif bash -c \
  "source /opt/dynabarn2/install/setup.bash && /opt/dynabarn2/src/dynabarn2/tools/learned/actor_loop.sh $RUN 0 1"
cat $RUN/logs/actor_*.log     # must end with a line like "dynabarn2_train/world_N,learned,succeeded,..."
ls $RUN/buffer                # one .npz file
```

If the log ends without a result line, look in `$RUN/logs/last_episode_*.log`.

## 2. Start training

Use a shared filesystem (scratch) that every node can see.

```bash
cd $HOME/DynaBARN/dyna_barn/ros2_pkg/ws/src/dynabarn2/tools/learned/cluster
export RUN=/scratch/$USER/dynabarn2_rl/run1
sbatch --export=ALL,RUN=$RUN,REPO=$HOME/DynaBARN,VENV=$HOME/dynabarn2_venv learner.slurm
sbatch --export=ALL,RUN=$RUN,SIF=/scratch/$USER/dynabarn2.sif actors.slurm
```

Check the partition names first: the scripts use `normal` (CPU) and `gpuq` (GPU), so run
`sinfo` and edit `#SBATCH --partition` if your cluster names them differently.

**How big.** On CPU nodes, each actor uses about 4 cores, because the lidar is drawn in
software. The script starts one actor per 4 cores. To change that, set `ACTORS_PER_NODE`.

`--array=0-3` means 4 nodes. More nodes collect data faster. Size the array to what the cluster
allows. The learner works the same with any number of actors, and actors can be added or
removed while it runs.

**Time limits.** Both jobs can simply be resubmitted. The learner resumes from
`checkpoint.pt` and reloads all saved episodes. Actors have no state.

## 3. Watch it

```bash
column -s, -t < $RUN/progress.csv | tail -5   # one row per 1000 training updates
ls $RUN/buffer | wc -l                          # episodes collected
tail $RUN/logs/actor_*_0.log                    # recent episodes of one actor
```

In `progress.csv`, `recent_success` is the success rate over the last 300 training episodes,
and `success_off/yield/block` splits it by obstacle mode. These episodes use random actions,
so the policy does better in testing. A healthy run:

- shows success rising over the first few hundred thousand transitions
- does not have `recent_collision` stuck near 1.0
- has `critic_loss` that does not grow without bound

## 4. Stop and hand back

```bash
touch $RUN/STOP          # the learner and all actors finish their current episode and exit
```

Send back:

- `$RUN/progress.csv`
- `$RUN/policy/` (all files, a few MB)
- `$RUN/checkpoint.pt`

The buffer folder is large and is not needed.

## 5. Evaluate (on the lab machine with Docker)

Pick a snapshot and run it like any other planner on the test sets. For example, here it is
under block on the confirmation set:

```bash
SYNC_START=true PLANNERS=learned LEARNED_POLICY=/path/to/policy_v200.npz \
  tools/run_baselines.sh learned_confirm_block 0 59 dynabarn2_confirm/world_ 3 4 block
```

Choose the snapshot using `worlds/dynabarn2` (the development set) only. Then report it once on
`dynabarn2_holdout` and `dynabarn2_confirm`.

## Settings, if needed

| Variable | Where | Default | Meaning |
|---|---|---|---|
| `ACTORS_PER_NODE` | actors.slurm | cpus / 4 | actors per node |
| `USE_GPU` | actors.slurm | unset | `1` on GPU nodes: lidar drawn on the GPU, so more actors per node (also add `--gres=gpu:1`) |
| `TRAIN_MODES` | actors.slurm | `off yield block` | obstacle modes sampled per episode |
| `EPISODE_TIME` | actors.slurm | 60 | time limit per training episode (s) |
| `FRAME_STRIDE` | both | 1 | control steps between the 4 stacked scans (2 = 0.6 s of motion); set the same for both jobs |
| `LEARNER_ARGS` | learner.slurm | | e.g. `--warmup 50000 --utd 2` (see `learner.py --help`) |
