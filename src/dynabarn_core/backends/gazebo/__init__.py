"""Gazebo (gz-sim, Harmonic or newer) backend.

Exporting is pure Python (no Gazebo needed): a scenario becomes ``world.sdf``
plus ``movers.yaml`` (trajectories of the moving obstacles) and
``bridge.yaml`` (a ros_gz_bridge config that gives the same ROS topics as
the MuJoCo node). Gazebo runs out of process: ``launch_command()`` starts it.

Moving obstacles are driven by ``dynabarn_core.backends.gazebo.mover``, a
Gazebo *Python* system that evaluates the very same trajectory objects as
MuJoCo, so obstacles follow identical paths in both simulators.
"""
from __future__ import annotations

from ..base import BACKENDS, Artifacts, Capabilities, SimBackend
from .exporter import GazeboExporter


@BACKENDS.register("gazebo")
class GazeboBackend(SimBackend):
    capabilities = Capabilities(
        in_process=False,
        sensors=frozenset({"lidar", "imu", "odometry", "joint_states", "ground_truth", "camera", "contact"}),
        obstacle_shapes=frozenset({"cylinder", "box", "sphere", "polygon"}),
        native_trajectories=frozenset({"static", "waypoints", "polynomial", "path"}),  # via the Python mover
    )
    exporter_cls = GazeboExporter

    def launch_command(self, artifacts: Artifacts, headless: bool = False, paused: bool = False) -> list[str]:
        cmd = ["gz", "sim"]
        if not paused:
            cmd.append("-r")
        if headless:
            cmd += ["-s", "--headless-rendering"]
        return cmd + [str(artifacts.scene)]
