# DynaBARN II on ROS 2

Dynamic-obstacle navigation worlds for [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) and Gazebo Harmonic, built to extend [The BARN Challenge](https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html). It contains the 60 original [DynaBARN](https://cs.gmu.edu/~xxiao2/Research/DynaBARN/DynaBARN.html) worlds, a new DynaBARN II world set, and a runner that scores a Clearpath Jackal with the BARN metric. Everything runs inside a Docker container.

## Repository layout

```
dyna_barn/ros2_pkg/
├── docker_env/             # Docker image and helper script
└── ws/src/
    ├── dynabarn2/          # ROS 2 package: plugin, worlds, generators, runner, baselines
    └── DynaBARN/           # Legacy Gazebo Classic generators (ignored by colcon)
```

Inside `dynabarn2/`:

| Path | Contents |
| --- | --- |
| `src/ObstacleMover.cc` | Gazebo system that moves an obstacle along timed waypoints |
| `worlds/dynabarn/` | The 60 original worlds, converted to Harmonic |
| `worlds/dynabarn2/` | The DynaBARN II set and its `manifest.json` |
| `dynabarn2_worlds/` | World generators and difficulty checks, as a Python package |
| `scripts/` | `trial_runner.py` and `ros2 run` wrappers for the generators |
| `launch/` | `run_trial.launch.py`, `nav2_baseline.launch.py` |
| `config/robot.yaml` | Jackal with one front Hokuyo lidar |
| `data/` | Original trajectories and the motion check against Gazebo Classic |
| `results/` | Baseline results |
| `tools/` | Batch runner, collision check, and extraction of the original trajectories |

## Requirements

- Docker with the Compose plugin
- An NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/): the lidar is GPU-rendered
- An X11 display, only for the Gazebo GUI

## Getting started

Pick the route that fits what you want to do.

| You want to | Use | Needs |
| --- | --- | --- |
| Run or submit a planner | Docker or Apptainer image | Docker, NVIDIA GPU |
| Develop on the code in ROS 2 | Dev container or `colcon build` | Docker, or ROS 2 Jazzy |
| Only generate worlds or compute difficulty | `pip install` | Python 3.10+ |

### Ready-to-run image

Run these from `dyna_barn/ros2_pkg/docker_env`. The first builds the base image, which takes a while.

```bash
docker compose build
docker build -f Dockerfile.release -t dynabarn2 ..
docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -v $PWD:/out dynabarn2 \
  ros2 launch dynabarn2 run_trial.launch.py world_file:=dynabarn2/world_0 out_file:=/out/results.csv
```

The BARN Challenge submits in Apptainer/Singularity format:

```bash
apptainer build dynabarn2.sif dynabarn2.def
apptainer run --nv dynabarn2.sif ros2 launch dynabarn2 run_trial.launch.py world_file:=dynabarn2/world_0
```

### ROS 2 development

```bash
cd dyna_barn/ros2_pkg/docker_env
./start.sh --getin                     # build and start the container, then open a shell
cd /ws && colcon build && source install/setup.bash
```

On your own ROS 2 Jazzy machine, copy `dynabarn2` into a workspace, then run `rosdep install --from-paths src --ignore-src -y` and `colcon build`.

If `nvidia-smi` fails in a long-running container with `Failed to initialize NVML`, restart the container.

### Generators only

```bash
pip install ./dyna_barn/ros2_pkg/ws/src/dynabarn2
dynabarn2-generate --save_dir worlds --per_cell 1 --families crowd --levels easy
```

This needs no ROS or Gazebo, but running the worlds needs the plugin from one of the routes above.

## Run a trial

```bash
ros2 launch dynabarn2 run_trial.launch.py world_file:=dynabarn2/world_0 planner:=mppi
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `world_file` | `dynabarn/world_0` | World under `worlds/`, without `.sdf` |
| `planner` | `mppi` | `mppi`, `dwb`, or `none` to run your own stack |
| `gui` | `false` | Open the Gazebo GUI |
| `out_file` | `results.csv` | CSV the result is appended to |
| `reactive` | `off` | `off` for fixed motion, `yield` or `block` for obstacles that respond to the robot |
| `timeout` | `100` | Time limit in seconds |

Use `planner:=none` for your own navigation stack. Subscribe to `/sensors/lidar2d_0/scan` and `/platform/odom/filtered`, and publish `geometry_msgs/TwistStamped` on `/platform/cmd_vel`. The robot starts at (11, 0) facing −x and the goal is (−9, 0), 20 m ahead.

### Scoring

The rules follow The BARN Challenge. The clock starts when the robot first moves. A trial ends when the robot is within 1 m of the goal, touches anything, or reaches the time limit.

```
score = success × optimal_time / clip(time, 2 × optimal_time, 8 × optimal_time)
optimal_time = path length / 2 m/s
```

The best possible score is 0.5. Each trial ends with one of these statuses:

| Status | Meaning |
| --- | --- |
| `succeeded` | Reached the goal |
| `collided` | Touched an obstacle or wall |
| `timeout` | Hit the time limit |
| `hit_at_start` | Hit while parked, before the robot moved |
| `no_start` | The robot never moved |

To see a world without a robot, use `gz sim -r <path to .sdf>`; Gazebo opens paused without `-r`.

## Worlds

**DynaBARN (`worlds/dynabarn/`)** holds the 60 original worlds. Their waypoints were recovered from the original compiled plugins; see [`tools/classic_extract`](dyna_barn/ros2_pkg/ws/src/dynabarn2/tools/classic_extract/README.md). Motion is kept faithful to Gazebo Classic, including its speed spikes, spline overshoot, and obstacles passing through each other.

**DynaBARN II (`worlds/dynabarn2/`)** holds 60 worlds: 5 families × 3 levels × 4 worlds.

| Family | Motion |
| --- | --- |
| `crowd` | Discs wander and bounce |
| `lanes` | Two-way lanes |
| `traffic` | Lanes with lane changes |
| `stop_and_go` | Traffic that keeps stopping and starting |
| `mixed` | Mixed sizes and shapes moving through static clutter |

Obstacles are simulated offline as colliding discs and saved as waypoints, so they bounce off each other while every run stays identical. Each world is kept only if a robot can reach the goal. `manifest.json` records its family, level, seed and difficulty, including `blind_collision`: the chance that a robot driving straight at the goal without reacting gets hit.

## Generating worlds

```bash
# The fixed DynaBARN II set; the same seed gives the same worlds
ros2 run dynabarn2 generate_dynabarn2.py --save_dir worlds/dynabarn2 --per_cell 4

# Custom one-off worlds, without obstacle-to-obstacle collisions
ros2 run dynabarn2 generate_movers.py --movement traffic --axis row --count 20 --save_dir /tmp/w
ros2 run dynabarn2 generate_polynomial.py --save_dir /tmp/poly --n_worlds 5 \
  --min_speed 0.5 --max_speed 1.5 --min_std 0.05 --max_std 0.2 --min_order 3 --max_order 6
```

Use `--help` on any generator for the full list of options. After `pip install`, the same tools are `dynabarn2-generate`, `dynabarn2-movers` and `dynabarn2-polynomial`. For fair comparisons, use the fixed sets rather than newly generated worlds.

## Baselines

```bash
# <tag> <first world> <last world> <world prefix> <repeats> <parallel trials> [off|yield|block]
dyna_barn/ros2_pkg/ws/src/dynabarn2/tools/run_baselines.sh dynabarn2 0 59 dynabarn2/world_ 3 6
```

Run this on the host, not in the container; it needs a built workspace. Finished trials are skipped, so an interrupted batch can be re-run. Results go to `results/<tag>.csv`. `tools/check_collisions.py` checks that every reported collision really happened next to something.

Baselines are Nav2 with the DWB and MPPI controllers on Clearpath's J100 configuration, made mapless and capped at 1 m/s. They are reference points, not tuned for dynamic obstacles.

## License

`dynabarn2` is released under the BSD-3-Clause license (see `package.xml`).
