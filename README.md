# dynabarn-core

A Python package for [DynaBARN](https://cs.gmu.edu/~xxiao2/Research/DynaBARN/DynaBARN.html)-style navigation among moving obstacles. It generates scenarios, simulates them in **MuJoCo** (the default), and streams the robot's sensors. It's meant as the base for DynaBARN projects such as ROS 2 nodes, learning setups and benchmarks: they `pip install dynabarn-core` and use `DynaBarn.get(...)` / `set_command(...)`, without any ROS or simulator code living in this package.

```python
from dynabarn_core import DynaBarn

db = DynaBarn.from_preset("burger_lanes", seed=1).start()   # simulation runs in the background
db.set_command(0.15, 0.0)
imu, scan = db.get("imu"), db.get_data("lidar")
```

| Simulator backend | Status |
| --- | --- |
| MuJoCo | Default. Export to MJCF and in-process simulation with lidar, IMU, odometry, joint states, contacts and cameras |
| gz-sim (modern Gazebo), Isaac Sim | Not yet; they plug in as backends (see "Adding a simulator") |

The scenario, asset and generator layers have no simulator or ROS imports, so they work the same on every ROS 2 distro.

## Quick start

```bash
pip install -e ".[dev]"              # from a clone; installs MuJoCo too

dynabarn list                                   # backends, generators, presets, trajectories, assets
dynabarn options lanes                          # a generator's options and defaults
dynabarn generate --preset burger_lanes --seed 7 -o out/burger.yaml          # from a preset
dynabarn generate lanes -s lanes=7 -s count=28 --seed 7 -o out/lanes.yaml    # or set options yourself
dynabarn view out/lanes.yaml --linear 0.5       # on-screen viewer; drive with the arrow keys
dynabarn run out/lanes.yaml --linear 0.5 --duration 60   # headless smoke run
dynabarn export out/lanes.yaml -b mujoco -o out/mujoco   # just write scene.xml
pytest
```

Lower-level Python API, for stepping a simulation yourself:

```python
from dynabarn_core import create_generator, create_backend, VelocityCommand

scenario = create_generator("lanes", lanes=6, count=36, speed=1.2).build(seed=7)
backend = create_backend("mujoco")
artifacts = backend.export(scenario, "out/mujoco")

with backend.create_simulation(artifacts, control_hz=20) as sim:
    obs = sim.reset()
    while obs.time < 60 and not obs.in_collision:
        obs = sim.step(VelocityCommand(linear=0.2, angular=0.0))
    print(obs.pose, obs.scan.ranges[0], obs.contacts)
```

## Running a scenario: `DynaBarn`

`DynaBarn` runs a scenario in a **background thread, in real time**. Your code reads any sensor's latest value whenever it likes and sends velocity commands. Nothing waits on anything else:

```python
from dynabarn_core import DynaBarn

db = DynaBarn.from_preset("burger_lanes", seed=1)   # or DynaBarn("my_scenario.yaml")
db.start()

db.set_command(0.15, 0.0)          # m/s, rad/s
imu  = db.get("imu")               # SensorReading: .data (Imu), .stamp (sim time), .frame
scan = db.get_data("lidar")        # just the payload (LaserScan)
odom = db.get_data("odom")         # Odometry in the odom frame (starts at 0, 0, 0)
db.stop()
```

The background thread advances physics in slices no longer than the fastest enabled sensor's period. So `get("imu")` really delivers a fresh 200 Hz sample every 5 ms, measured as 401 samples in 2 s.

| Getters | |
| --- | --- |
| `get(name)` / `get_data(name)` | Latest reading or payload of a sensor (None before the first one) |
| `wait_for(name, newer_than=None, timeout=1.0)` | Block until the sensor has a newer reading |
| `sensors()` | Every sensor: kind, frame, rate, enabled, **mount pose relative to the base frame**, full config |
| `observe()` | Snapshot: ground-truth pose and velocity, collisions, latest of every sensor |
| `time`, `realtime_factor`, `running`, `paused` | |

| Setters and control | |
| --- | --- |
| `set_command(linear, angular)` | Velocity command |
| `set_sensor_enabled(name, bool)` | Switch a sensor on or off at runtime |
| `configure_sensor(name, **changes)` | e.g. `configure_sensor("lidar", noise_std=0.02, rate_hz=10)` or `configure_sensor("odom", source="wheels")` |
| `subscribe(name or "*", callback)` | Callback for every new reading, on the simulation thread; returns an unsubscribe function |
| `start()`, `stop()`, `pause()`, `resume()`, `reset()` | |
| `step(command)` | Synchronous stepping instead of the thread, e.g. for RL or tests |

Options: `DynaBarn(..., realtime=1.0)` (`2.0` runs twice as fast, `None` as fast as possible). `command_timeout=0.5` stops the robot when commands stop arriving. Sensor overrides: `enable=["rgb"], disable=["imu"]`, `seed=...` (noise), `control_hz=...`.

### Sensors

Each robot's `asset.yaml` lists its sensors. **Realistic extras are off by default**; switch them on in the YAML or at runtime:

```yaml
sensors:
  lidar:    {frame: base_scan, samples: 360, ..., rate_hz: 5, noise_std: 0.0}   # m
  imu:      {frame: imu_link, rate_hz: 200, gyro_noise_std: 0.0, accel_noise_std: 0.0}
  odometry: {frame: odom, rate_hz: 30, source: ground_truth,                  # or wheels
             linear_noise_std: 0.0, angular_noise_std: 0.0}
  joint_states: {rate_hz: 30}
  contact:      {rate_hz: 50}
  ground_truth: {rate_hz: 50}       # exact world pose, for evaluation
  cameras:
    - {name: rgb,   ..., rate_hz: 30, enabled: false}
    - {name: depth, ..., rate_hz: 30, enabled: false, depth: true}
```

| Sensor (`get` name) | Payload | Notes |
| --- | --- | --- |
| `lidar` | `LaserScan` | `inf` where nothing is within range |
| `imu` | `Imu` | Orientation, angular velocity, linear acceleration (includes gravity: +9.81 z at rest) |
| `odom` | `Odometry` | `ground_truth`: exact, relative to the start pose. `wheels`: integrated wheel rotation, which drifts when wheels slip (Jackal turns) |
| `joint_states` | `JointStates` | Wheel positions and velocities |
| `contact` | `Contacts` | What the robot is touching |
| `ground_truth` | `GroundTruth` | World pose and velocity |
| `rgb`, `depth` | `Image` | `rgb8` (H×W×3 uint8) / `32FC1` (H×W float32, metres). Need OpenGL (`MUJOCO_GL=egl` on headless machines) |

**Camera cost.** Cameras render on the simulation thread. Measured with both 640×480 cameras at 30 Hz:

| | Burger preset | Jackal preset |
| --- | --- | --- |
| GPU rendering (RTX 4060 Ti, `MUJOCO_GL=glfw`): max speed, cameras off / rgb / rgb + depth | 40× / 16× / 16× real time | 24× / 12× / 8× |
| GPU rendering, real-time mode | Stays real time, IMU 200 Hz | Stays real time, IMU 200 Hz |
| CPU rendering (`MUJOCO_GL=osmesa`): max speed, rgb / rgb + depth | 3.1× / 2.6× | 1.6× / 1.3× |
| CPU rendering, real-time mode | Stays real time, but `get("imu")` sees ~170 of 200 samples/s | Stays real time, `get("imu")` sees ~140 of 200 samples/s |

With a GPU, cameras only matter when running faster than real time. With CPU rendering, each frame takes 9–19 ms. Sensors still produce every sample in simulation time, but they arrive in bursts after each frame, so a `get()` poller misses some. Use `subscribe()` to receive every reading, or lower the camera `rate_hz` or resolution.

### Using it from a ROS 2 node (not included, not tested)

dynabarn has no ROS dependency. A node only needs `get` and `set_command`, which is the pattern this API is designed for:

```python
class DynaBarnNode(Node):
    def __init__(self):
        super().__init__("dynabarn")
        self.db = DynaBarn.from_preset("burger_lanes").start()
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.create_subscription(Twist, "cmd_vel", lambda m: self.db.set_command(m.linear.x, m.angular.z), 10)
        self.create_timer(1 / 200, self.publish_imu)        # any rate you like
        self.last = None

    def publish_imu(self):
        r = self.db.get("imu")
        if r is None or r is self.last:                       # only publish new samples
            return
        self.last = r
        self.imu_pub.publish(to_ros_imu(r.data, stamp=r.stamp, frame_id=r.frame))
```

`sensors()` gives each sensor's mount pose, which is what a static TF broadcaster needs.

## Robots and scale

DynaBARN's arena and obstacles were designed for the Clearpath Jackal, so the generator defaults are a Jackal in the 20 x 20 m arena. For the much smaller TurtleBot3 Burger, use the `burger_*` presets: an 8 x 8 m arena with 0.1 m obstacles, scaled by the robots' size ratio (about 0.4).

| Preset | Robot | Arena | Obstacles |
| --- | --- | --- | --- |
| `dynabarn_lanes` | Jackal (0.51 x 0.43 m, skid steer, up to 2 m/s) | 20 x 20 m | 36 x r 0.25 m, 6 lanes, 1 m/s |
| `dynabarn_random` | Jackal | 20 x 20 m | 36 x r 0.25 m, random walks, 0.5-1.5 m/s |
| `burger_lanes` | TurtleBot3 Burger (0.21 m, up to 0.22 m/s) | 8 x 8 m | 18 x r 0.1 m, 6 lanes, 0.15 m/s |
| `burger_random` | TurtleBot3 Burger | 8 x 8 m | 16 x r 0.1 m, random walks, 0.08-0.2 m/s |

Presets are YAML files in `src/dynabarn_core/data/presets/`. Pass your own with `--preset path/to/file.yaml`, or override any option: `dynabarn generate --preset burger_lanes -s count=24 ...`.

## Obstacles never overlap

Moving obstacles follow scripted paths and do not collide with each other in any simulator, so the generators guarantee they never overlap:

- `lanes`: obstacles sharing a lane each get their own section of it, with `lane_gap` metres between surfaces.
- `random`, `layout=cells` (default): the area is split into a grid with one obstacle per cell, and each obstacle moves randomly inside its own cell.
- `random`, `layout=free`: obstacles roam the whole area, and paths are re-drawn until they keep their distance. This only works for a handful of obstacles, and you get a clear error when it can't fit them.

`min_clearance` (default 0 m) adds extra space between obstacle surfaces, checked over `check_horizon` seconds (default 120 s). `min_clearance=none` allows overlaps, as in the original DynaBARN.

## Architecture

```
src/dynabarn_core/
  core/         Pose2D, Bounds, shapes, trajectories, Scenario, messages, Registry   (no ROS, no simulators)
  assets/       asset.yaml manifests -> Robot/Obstacle/Environment dataclasses, search path
  generators/   Scenario generators (lanes, random, empty)
  importers/    world importers (sdf, world_collection)
  backends/     simulator adapters: SimBackend = Exporter + runtime (Simulation or launch_command)
    mujoco/     MuJoCo exporter, simulation, sensors, viewer
  runtime.py    DynaBarn: background thread, get() / set_command()
  data/assets/  built-in robots, obstacles and environments (shipped in the wheel)
  data/presets/ built-in presets
scenarios/      example scenario files
examples/       using dynabarn from your own code
```

Each layer depends only on the layers listed above it.

| Pattern | Where | Why |
| --- | --- | --- |
| Frozen dataclasses | `core`, `assets` | Plain, immutable, serialisable data |
| Strategy | `core.trajectories` | New motion types without touching backends |
| Registry + Factory | `core.registry`; `create_generator`, `create_backend` | Pick implementations by name (CLI, config files) |
| Entry points | groups `dynabarn_core.backends`, `dynabarn_core.generators`, `dynabarn_core.trajectories`, `dynabarn_core.asset_kinds` | Add a simulator or generator from a separate package |
| Adapter | `backends/*` | One adapter per simulator, all with the same interface |
| Template Method | `Generator.build` | Generators only write `make_trajectories` |

Two rules keep the backends interchangeable:

- **Trajectories have a universal export form.** Every trajectory implements `pose_at(t)` and `as_waypoints()`. A simulator that can't call Python at runtime, such as a gz-sim C++ plugin, plays back the waypoints. Backends may also evaluate known types natively.
- **Simulations speak `core.messages`.** In-process backends implement `Simulation.reset/step/observe` using `VelocityCommand` and `Observation`. Middleware such as ROS 2 or a Gym wrapper converts these types, so simulators never import ROS and ROS code never imports a simulator.

## Assets

Robots, obstacles and environments are **assets**, looked up by `id`. The defaults ship inside the package (`src/dynabarn_core/data/assets/`), so `pip install` includes them. `dynabarn list assets` shows what's available.

You can add or replace assets in three ways. For a given id, the first match in this order wins, so reusing a built-in id overrides it:

**1. In code**, with no files needed:

```python
from dynabarn_core import AssetLibrary
from dynabarn_core.assets import ObstacleAsset, EnvironmentAsset, Wall
from dynabarn_core.core import Cylinder, Box

assets = AssetLibrary()
assets.add(
    ObstacleAsset(id="green_barrel", geometry=Cylinder(radius=0.3, length=0.9), color=(0.1, 0.6, 0.2, 1)),
    EnvironmentAsset(id="corridor", ground_size=(30, 12),
                     walls=(Wall("left", 0, 5, 0, (24, 0.2, 1)), Wall("right", 0, -5, 0, (24, 0.2, 1)))),
)
assets.add_manifest({"id": "cone", "kind": "obstacle", "geometry": {"type": "sphere", "radius": 0.2}})

gen = create_generator("lanes", assets=assets, obstacle_asset="green_barrel")
backend = create_backend("mujoco", assets)
```

**2. A folder of `asset.yaml` files**, passed as `AssetLibrary(["my_assets"])`, `assets.add_folder("my_assets")`, `dynabarn --assets my_assets ...` or `DYNABARN_ASSET_PATH=my_assets`. The easiest start is to copy a built-in asset and edit it:

```bash
dynabarn assets copy cylinder_small my_assets --as tall_post   # new asset 'tall_post'
dynabarn assets copy jackal my_assets                          # same id: overrides the built-in jackal
dynabarn --assets my_assets generate --preset burger_lanes -s obstacle_asset=tall_post -o run.yaml
```

**3. The built-in defaults.**

A manifest (`asset.yaml`) looks like this:

```yaml
id: cylinder_red
kind: obstacle                  # robot | obstacle | environment
geometry: {type: cylinder, radius: 0.25, length: 0.5}
color: [0.8, 0.1, 0.1, 1.0]
representations: {}             # optional per-simulator files
```

Robots also describe their drive and sensors, plus one `representations` entry per simulator. `bindings` map dynabarn's generic names to the names inside that simulator's file, so you can swap in a different model without changing code. A side can list several motors, as with skid steer:

```yaml
representations:
  mujoco:
    path: mujoco/robot.xml       # relative to the asset folder
    bindings: {root_body: base_link, left_actuator: front_left_vel rear_left_vel,
               right_actuator: front_right_vel rear_right_vel, lidar_site: base_scan}
```

Scenarios reference assets by id, so the same scenario file works with different asset sets.

## Importing worlds from other tools

Worlds made for other tools can be imported as environment assets, which every backend can then build. Importers keep **geometry and metadata only**. Simulator-specific parts such as Gazebo system plugins are dropped, because each backend adds what its simulator needs when it exports.

| Importer | Reads |
| --- | --- |
| `sdf` | Any SDF world whose static models use boxes, cylinders, spheres or polylines (extruded polygons). Anything it can't represent (meshes, `<include>`, tilted shapes, non-static models) is reported as a warning, never dropped silently |
| `world_collection` | A world folder from [mertgulerx/world-collection](https://github.com/mertgulerx/world-collection) (Apache-2.0): the SDF plus `world.yaml` start poses, free-space outline, doors and measured difficulty values. Copies the world's LICENSE along with it |

```bash
git clone --depth 1 https://github.com/mertgulerx/world-collection
dynabarn import-world world-collection/worlds -o wc_assets       # all 43 worlds in about a second
dynabarn --assets wc_assets list assets
dynabarn --assets wc_assets generate empty -s environment=wc_sc4_office_strip -s spawn=S2 -o sc4.yaml
dynabarn --assets wc_assets view sc4.yaml
```

In Python: `res = import_world("path/to/world")`, then `AssetLibrary().add(res.environment)`, or `write_environment(res, "my_assets")`.

- **`empty` generator:** just the environment and robot, starting at the world's first spawn unless `spawn=` or `start_x`/`start_y` are given.
- **Our moving obstacles inside imported maps:** not yet. Placing them needs the free-space outline (next step), as do the world-collection moving objects (`MO1`, `MO2`), which are kept in the metadata for now.
- **Concave polygons:** MuJoCo collides meshes as convex hulls, so concave polygons are split into convex prisms automatically.

**Checked against all 43 world-collection worlds:**
- Every world imports, and object counts match the SDF.
- At every spawn, both robots start clear of walls and rest on the floor.
- The free floor area computed from the imported geometry matches the author's measured value to 0.00% in 42 worlds. The 500 m maze differs by 0.14%, from the coarser check grid; the author's tolerance is 2%.

## Using it in your own project

```bash
pip install dynabarn-core            # not on PyPI yet: pip install "git+<repo-url>" or pip install /path/to/repo
```

[`examples/quickstart.py`](examples/quickstart.py) is a complete example covering custom assets in code, a generated scenario, and a simple controller loop that stops in front of obstacles. It runs headless or with `--view`. The loop is the integration point for your own planner:

```python
with backend.create_simulation(artifacts, control_hz=20) as sim:
    obs = sim.reset()
    while not done:
        cmd = my_planner(obs.pose, obs.velocity, obs.scan)   # -> VelocityCommand(linear, angular)
        obs = sim.step(cmd)                                   # obs.contacts lists collisions
```

## Adding a simulator

1. Write an `Exporter` that turns `Scenario` + `AssetLibrary` into the simulator's files.
2. Subclass `SimBackend` and set `capabilities` and `exporter_cls`. Then implement `create_simulation`, for an in-process simulator such as Isaac Sim, or `launch_command`, for an external one such as gz-sim.
3. Register it with `@BACKENDS.register("name")`, or as an entry point in your own package:
   ```toml
   [project.entry-points."dynabarn_core.backends"]
   isaac = "my_pkg.isaac:IsaacBackend"
   ```
4. Add the robot's file for that simulator under `representations` in its `asset.yaml`.

`SimBackend.check()` validates a scenario before export and lists any missing assets or representations.

## Notes

- The MuJoCo robot models use primitive shapes with the real dimensions and masses. The Burger tracks velocity commands to within about 2%. The Jackal is skid steer: in MuJoCo it turns at about 43% of what its physical 0.376 m track predicts, so its `asset.yaml` uses an effective `wheel_separation` of 0.87 m. With that it tracks within about 5%.
- The original DynaBARN lane generator ran every obstacle in a lane along the same path, so they passed through each other. Its `passes` option is gone because the motion loops forever.
