"""MuJoCo backend. ``mujoco`` is imported only when a simulation is created,
so exporting MJCF works without MuJoCo installed."""
from __future__ import annotations

from ..base import BACKENDS, Artifacts, Capabilities, SimBackend, Simulation
from .exporter import MujocoExporter


@BACKENDS.register("mujoco")
class MujocoBackend(SimBackend):
    capabilities = Capabilities(
        in_process=True,
        sensors=frozenset({"lidar", "rgb", "depth", "contact", "odom"}),
        native_trajectories=frozenset({"static", "waypoints", "polynomial"}),  # evaluated in Python
    )
    exporter_cls = MujocoExporter

    def create_simulation(self, artifacts: Artifacts, **options) -> Simulation:
        from .sim import MujocoSimulation

        from ...core.scenario import Scenario

        scenario = Scenario.load(artifacts.scenario)
        return MujocoSimulation(artifacts, self.assets.robot(scenario.robot.asset), **options)

    def view(self, artifacts: Artifacts, control_hz: float = 20.0, **options) -> None:
        from .viewer import run_viewer

        with self.create_simulation(artifacts, control_hz=control_hz) as sim:
            run_viewer(sim, **options)
