# What porting to ROS 2 + Gazebo taught us about the API

Notes from building `ros2/dynabarn_ros` and the Gazebo backend on ROS 2 Jazzy +
Gazebo Harmonic (`feature/ros2-gazebo`). Each item: what we found, what changed.

## Changed in dynabarn-core

1. **Sensor mounts must not live only inside a simulator's model file.**
   They were read from MuJoCo sites, so neither Gazebo nor ROS TF could use them.
   Now every sensor has `mount: [x, y, z, roll, pitch, yaw]` in `asset.yaml`.
   `tests/test_gazebo_export.py` checks it against the MuJoCo model.

2. **Odometry is shared logic, not simulator logic.**
   MuJoCo computed odometry itself. Gazebo's `OdometryPublisher` reports the
   world pose, while ours starts at the robot's start pose, as ROS expects.
   The estimator (ground truth relative to the start, or integrated wheels
   with slip and noise) moved to `core/odometry.py`. Both backends now feed it
   true poses and wheel angles, so they report identical odometry for
   identical motion.

3. **Drive calibration is per simulator.**
   A skid-steer robot's effective track width depends on the physics
   engine's friction model: 0.87 m in MuJoCo, 0.525 m in Gazebo/DART.
   Representations now have an optional `drive:` override, and
   `RobotAsset.drive_for(backend)` applies it.

4. **One description of the ROS interface.**
   `dynabarn_core/interface.py` lists the topics, message types and frames
   for a robot. It doesn't import ROS. The MuJoCo node publishes from it, and
   the Gazebo exporter writes `bridge.yaml` from it, so the two can't drift
   apart. `test_backend_parity.py` checks this through ROS.

5. **Shared logic runs inside Gazebo, so nothing is written twice.**
   Gazebo Harmonic loads Python systems. `backends/gazebo/system.py` moves the
   obstacles with our own trajectory objects, so the paths are bit-identical
   to MuJoCo (checked on 2,600+ samples to within 1e-6 m). It also runs the
   shared odometry. There's no C++ plugin to build per Gazebo version.

6. **`DynaBarn.open_viewer()`.**
   Anything that runs the simulation in the background, such as the ROS
   node, can now show it on screen.

7. **Cameras have an optical frame.**
   ROS images are stamped in an optical frame (z forward). `CameraSpec.optical_frame`
   and the static TF from `interface.static_frames()` supply it for both
   simulators.

## Packaging lessons (ROS + pip)

* **ROS executables and venvs.** colcon writes `#!/usr/bin/python3` unless
  colcon itself runs under the venv's Python. The image runs
  `python -m colcon build` from a venv created with `--system-site-packages`.
  Then executables find both `rclpy` and `dynabarn-core`.
* **Launch files must not import pip packages.** `ros2 launch` always runs the
  system Python. `gazebo.launch.py` therefore runs the `prepare_gazebo`
  executable to export the world and reads its JSON output.
* **Exported files contain absolute paths** (meshes, configs). Export them
  where the simulator will read them. The launch files do this.
* **Gazebo's embedded Python** must find `dynabarn_core`. The launch file adds
  it to `PYTHONPATH` for `gz sim`.

## Known differences between the simulators (by design or not yet done)

* **Physics.** Robot motion matches within a few percent once calibrated. Over
  8 s, end positions differ by under 15 cm (`test_gazebo_sim.py`).
* **Timestamps.** A MuJoCo reading stamped `t` shows the world at `t - dt`
  (one 2 ms physics step). Within a reading everything is consistent. Gazebo
  stamps with the exact time. This only matters for sub-step comparisons.
* **Runtime sensor changes.** `configure_sensor()` works in MuJoCo. In Gazebo,
  sensors are fixed when the world is exported. Change `asset.yaml` and relaunch.
* **Contacts** have no standard ROS message. They're available through the
  dynabarn API (MuJoCo) and the Gazebo contact topic, but not bridged.
* **`DynaBarn` (in-process API) is MuJoCo only.** Gazebo runs out of process and
  is used through ROS (or gz-transport). A gz-transport `Simulation` adapter
  would give Gazebo `DynaBarn.get()` too. It isn't needed for ROS users.
