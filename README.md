# DynaBARN on ROS 2

Tools for generating [DynaBARN](https://cs.gmu.edu/~xxiao2/Research/DynaBARN/DynaBARN.html)-style dynamic obstacle worlds for Gazebo Classic 11, plus a ROS 2 Humble package for running and benchmarking a TurtleBot3 in those worlds. Everything runs inside a Docker container.

## Repository layout

```
dyna_barn/ros2_pkg/
├── docker_env/                  # Docker image + helper script (ROS 2 Humble, Gazebo Classic, ros_gz)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh            # sources ROS/Gazebo and sets DynaBARN plugin/world paths
│   └── start.sh
└── ws/                          # colcon workspace, mounted at /ws in the container
    └── src/
        ├── DynaBARN/            # world + obstacle-plugin generators
        └── dynabarn_turtlebot/  # ROS 2 package: launch files, robot model, nodes
```

## Requirements

- Docker with the Compose plugin
- An X11 display on the host (Gazebo and rqt need a GUI)
- An NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) (the compose file requests `gpus: all`; remove that line to run without one)

## Getting started

### 1. Start the container

```bash
cd dyna_barn/ros2_pkg/docker_env
./start.sh            # build (if needed) and start the container
./start.sh --getin    # start (if needed) and open a shell inside it
```

Other options: `--rebuild` (rebuild the image with no cache), `--shell` (shell into a running container), `--restart`.

You can also enter the running container directly:

```bash
docker exec -it ros2-humble-gz /entrypoint.sh bash
```

Always go through `/entrypoint.sh` so that ROS, Gazebo and the DynaBARN plugin paths are set.

### 2. Generate a world

Generated files go in `DynaBARN/worlds/` and `DynaBARN/plugins/`, which are git-ignored. Inside the container:

```bash
cd /ws/src/DynaBARN
python3 generate_grid_plugins_gz11.py \
  --movement controlled --axis row \
  --rows 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 3 --speed 1.2 --count 36 --stagger_phase
```

This compiles one Gazebo plugin per obstacle into `plugins/build/` and writes `worlds/world.world`. Open a new shell (or re-run `/entrypoint.sh`) after the first generation so `GAZEBO_PLUGIN_PATH` picks up `plugins/build`.

See [DynaBARN generators](#dynabarn-generators) for the other motion modes.

### 3. Build the ROS 2 package

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Run

```bash
# Just the world
ros2 launch dynabarn_turtlebot bringup_dynabarn.launch.py world:=world.world

# World + TurtleBot3 with camera + go-to-goal controller + annotated image viewer
ros2 launch dynabarn_turtlebot end_to_end.launch.py goal_x:=10.0 goal_y:=0.0

# Full benchmark: red-obstacle detector, then the end-to-end run after a delay
ros2 launch dynabarn_turtlebot benchmarking.launch.py delay_sec:=5.0
```

## DynaBARN generators

All scripts live in `ws/src/DynaBARN/` and use `base_boilerplate.world` as a template (generated models replace its `TOKEN` placeholder).

| Script | What it generates |
| --- | --- |
| `generate_grid_plugins_gz11.py` | Lane-based (`--movement controlled`) or random-waypoint (`--movement random`) obstacles |
| `generate_plugins_gz11.py` | Original DynaBARN-style polynomial trajectories |
| `generate_dynabarn_traffic.py` | Traffic-like lanes with lane changes (work in progress) |
| `create_plugin_polynomial.py`, `polynomial_fit.py` | Polynomial trajectory helpers |

Examples:

```bash
# Front↔back columns, random constant speed per obstacle
python3 generate_grid_plugins_gz11.py \
  --movement controlled --axis column \
  --cols 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 3 --count 36 --stagger_phase \
  --speed_min 0.6 --speed_max 1.8 --speed_mode agent

# Random bounce-walk (reflects at borders)
python3 generate_grid_plugins_gz11.py \
  --movement random --rand_bounce --rand_points 30 \
  --speed_min 0.6 --speed_max 1.8 \
  --count 40 --stagger_phase

# Polynomial trajectories
python3 generate_plugins_gz11.py \
  --save_dir ./worlds --plugins_dir ./plugins --seed 11 \
  --min_object 2 --max_object 6 \
  --min_speed 0.5 --max_speed 1.5 \
  --min_std 0.05 --max_std 0.20 \
  --min_order 3 --max_order 6
```

Note: `generate_plugins_gz11.py` currently always writes 20 worlds, regardless of `--n_worlds`.

To view a world without ROS:

```bash
gazebo --verbose /ws/src/DynaBARN/worlds/world.world
```

More background and the 60 premade DynaBARN worlds are in [`ws/src/DynaBARN/README.md`](dyna_barn/ros2_pkg/ws/src/DynaBARN/README.md).

## `dynabarn_turtlebot` package

### Launch files

| Launch file | Description | Key arguments |
| --- | --- | --- |
| `bringup_dynabarn.launch.py` | Gazebo with a DynaBARN world | `worlds_dir`, `world` |
| `bringup_turtlebot.launch.py` | Stock TurtleBot3 in an empty world | `model`, `gui`, `x`, `y`, `z`, `yaw` |
| `bringup_turtlebotCamera.launch.py` | TurtleBot3 Burger with RGB/depth camera in an empty world | `gui`, `x`, `y`, `z`, `yaw` |
| `wander.launch.py` | DynaBARN world + camera TurtleBot (defaults to `world_18.world`) | `world`, `x`, `y`, `yaw` |
| `end_to_end.launch.py` | World + robot + `end_to_end` controller + `rqt_image_view` | `world`, `x` (default `-8.0`), `goal_x`, `goal_y` |
| `imagedetector.launch.py` | Red-obstacle detector node | — |
| `benchmarking.launch.py` | `imagedetector`, then `end_to_end` after a delay | `delay_sec` |

### Nodes

| Executable | Description | Topics |
| --- | --- | --- |
| `end_to_end` | Proportional go-to-goal controller that stops in front of close obstacles | sub `/odom`, `/scan`; pub `/cmd_vel` |
| `wander` | RRT-based local planner on an occupancy grid built from the laser scan | sub `/odom`, `/scan`; pub `/cmd_vel` |
| `imagedetector` | Detects red obstacles in RGB, estimates distance from depth | sub `/turtlebot3/rgb_cam/image_raw`, `/turtlebot3/depth_cam/image_raw`; pub `/imagedetector/image_annotated`, `/barn_distance` |

Run a node on its own with, for example:

```bash
ros2 run dynabarn_turtlebot end_to_end --ros-args -p goal_x:=5.0 -p max_linear_speed:=0.2
```

`collision_counter.py`, `collision_watch.py` and `bumper_topic_logger.py` are collision-monitoring helpers that subscribe to Gazebo bumper `ContactsState` topics. They are not registered as executables yet; run them with `python3` from the package directory.

To drive the robot manually:

```bash
sudo apt-get update && sudo apt-get install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## License

`dynabarn_turtlebot` is released under the BSD-3-Clause license (see `setup.py`).
