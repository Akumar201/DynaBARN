"""Simulator backends (Adapter + Abstract Factory).

A backend adapts dynabarn's simulator-independent model to one simulator.
It provides two things:

* an ``Exporter`` that turns a ``Scenario`` + assets into the files the
  simulator loads (MJCF, SDF, USD, ...). Pure file generation, easy to test.
* a runtime, which is one of:
    - ``Simulation`` (in-process: MuJoCo, Isaac Sim) with reset/step/observe,
      wrapped by middleware bridges such as the ROS 2 bridge node;
    - ``launch_command`` (out-of-process: gz-sim), where the simulator talks
      to ROS directly through its own bridge.

New simulators register with ``@BACKENDS.register("name")`` or the
``dynabarn_core.backends`` entry point group; nothing else in dynabarn changes.
"""
from __future__ import annotations

import shutil
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, ClassVar

from ..assets import AssetError, AssetLibrary, ObstacleAsset
from ..core.messages import Observation, SensorInfo, SensorReading, VelocityCommand
from ..core.registry import Registry
from ..core.scenario import Scenario
from ..core.shapes import Box, Cylinder, ExtrudedPolygon, Sphere

_SHAPE_NAMES = {Cylinder: "cylinder", Box: "box", Sphere: "sphere", ExtrudedPolygon: "polygon"}


class BackendError(RuntimeError):
    pass


@dataclass(frozen=True)
class Capabilities:
    in_process: bool
    sensors: frozenset[str] = frozenset()
    obstacle_shapes: frozenset[str] = frozenset({"cylinder", "box", "sphere"})
    # trajectory types evaluated natively; everything else is exported as waypoints
    native_trajectories: frozenset[str] = frozenset()


@dataclass(frozen=True)
class Artifacts:
    """What an exporter produced. ``scene`` is the file the simulator loads."""

    backend: str
    out_dir: Path
    scene: Path
    scenario: Path
    files: dict[str, Path] = field(default_factory=dict)


class Exporter(ABC):
    def __init__(self, backend: "SimBackend") -> None:
        self.backend = backend
        self.assets = backend.assets

    @abstractmethod
    def export(self, scenario: Scenario, out_dir: Path) -> Artifacts:
        ...


SensorCallback = Callable[[SensorReading], None]


class Simulation(ABC):
    """In-process runtime API shared by all in-process backends.

    Stepping:   ``reset()``, ``step(command)``, ``advance(seconds)``
    Getters:    ``latest(name)``, ``sensors()``, ``observe()``, ``time``
    Setters:    ``set_command()``, ``set_sensor_enabled()``, ``configure_sensor()``
    Streaming:  ``subscribe(name, callback)``: called for every new reading,
                stamped with simulation time, at the sensor's own rate.

    Backends implement the abstract methods and call ``_publish`` for each
    new sensor reading; everything else is provided here.
    """

    #: control period in seconds; one ``step`` advances the world by this much
    dt: float

    def __init__(self) -> None:
        self._latest: dict[str, SensorReading] = {}
        self._subscribers: dict[str, list[SensorCallback]] = {}
        self._command = VelocityCommand()
        self._lock = threading.RLock()

    # -- to implement ------------------------------------------------------
    @abstractmethod
    def _reset(self) -> None:
        """Reset the world and the sensors' internal state."""

    @abstractmethod
    def _advance(self, seconds: float) -> None:
        """Advance physics by ``seconds`` using ``self._command``, publishing
        each sensor reading as it falls due."""

    @abstractmethod
    def observe(self) -> Observation:
        ...

    @abstractmethod
    def sensors(self) -> dict[str, SensorInfo]:
        """Static description of every sensor (enabled or not), by name."""

    @abstractmethod
    def configure_sensor(self, name: str, **changes) -> SensorInfo:
        """Change a sensor's settings at runtime, e.g. ``rate_hz``,
        ``noise_std``, ``enabled``. Keys are the sensor's asset.yaml fields."""

    @property
    @abstractmethod
    def time(self) -> float:
        ...

    # -- stepping ------------------------------------------------------------
    def reset(self) -> Observation:
        with self._lock:
            self._latest.clear()
            self._command = VelocityCommand()
            self._reset()
            return self.observe()

    def step(self, command: VelocityCommand | None = None) -> Observation:
        """Advance one control period (``dt``). ``None`` keeps the last command."""
        with self._lock:
            if command is not None:
                self._command = command
            self._advance(self.dt)
            return self.observe()

    def advance(self, seconds: float) -> None:
        """Advance by an arbitrary duration with the current command."""
        with self._lock:
            self._advance(seconds)

    # -- getters / setters -----------------------------------------------------
    def latest(self, name: str) -> SensorReading | None:
        """Most recent reading of sensor ``name`` (None before the first one)."""
        if name not in self.sensors():
            raise KeyError(f"no sensor '{name}'. Sensors: {', '.join(self.sensors())}")
        return self._latest.get(name)

    @property
    def command(self) -> VelocityCommand:
        return self._command

    def set_command(self, command: VelocityCommand) -> None:
        self._command = command

    def set_sensor_enabled(self, name: str, enabled: bool) -> SensorInfo:
        return self.configure_sensor(name, enabled=enabled)

    # -- streaming -----------------------------------------------------------
    def subscribe(self, name: str, callback: SensorCallback) -> Callable[[], None]:
        """Call ``callback(reading)`` for every new reading of sensor ``name``
        (``"*"`` for all sensors). Returns a function that unsubscribes.
        Callbacks run on the simulation's thread and should return quickly."""
        if name != "*" and name not in self.sensors():
            raise KeyError(f"no sensor '{name}'. Sensors: {', '.join(self.sensors())}")
        self._subscribers.setdefault(name, []).append(callback)

        def unsubscribe() -> None:
            subs = self._subscribers.get(name, [])
            if callback in subs:
                subs.remove(callback)

        return unsubscribe

    def _publish(self, reading: SensorReading) -> None:
        self._latest[reading.name] = reading
        for cb in (*self._subscribers.get(reading.name, ()), *self._subscribers.get("*", ())):
            cb(reading)

    def open_viewer(self, follow: bool = False):
        """Open an on-screen window showing the simulation, for programs that
        drive it themselves (e.g. ``DynaBarn`` in the background). Returns a
        handle with ``sync()`` (call with the simulation lock held),
        ``is_running()`` and ``close()``."""
        raise BackendError(f"{type(self).__name__} has no on-screen viewer")

    def release_thread_resources(self) -> None:
        """Free resources tied to the calling thread, such as OpenGL contexts
        used for camera rendering. Runners call this on the old thread before
        a different thread starts driving the simulation."""

    def close(self) -> None:
        pass

    def __enter__(self) -> "Simulation":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class SimBackend(ABC):
    registry_name: ClassVar[str]
    capabilities: ClassVar[Capabilities]
    exporter_cls: ClassVar[type[Exporter]]

    def __init__(self, assets: AssetLibrary | None = None) -> None:
        self.assets = assets or AssetLibrary()

    @property
    def name(self) -> str:
        return self.registry_name

    # -- validation ------------------------------------------------------
    def check(self, scenario: Scenario) -> list[str]:
        """Return human-readable problems; empty means the scenario is supported."""
        problems: list[str] = []
        try:
            self.assets.robot(scenario.robot.asset).require(self.name)
        except AssetError as e:
            problems.append(str(e))
        try:
            self.assets.environment(scenario.environment)
        except AssetError as e:
            problems.append(str(e))
        for asset_id in sorted({o.asset for o in scenario.obstacles}):
            try:
                ob: ObstacleAsset = self.assets.obstacle(asset_id)
            except AssetError as e:
                problems.append(str(e))
                continue
            shape = _SHAPE_NAMES[type(ob.geometry)]
            if shape not in self.capabilities.obstacle_shapes and ob.representation(self.name) is None:
                problems.append(f"backend '{self.name}' cannot build {shape} obstacles ({asset_id})")
        return problems

    # -- export ------------------------------------------------------------
    def export(self, scenario: Scenario, out_dir: str | Path, clean: bool = False) -> Artifacts:
        problems = self.check(scenario)
        if problems:
            raise BackendError(f"scenario '{scenario.name}' is not supported by '{self.name}':\n  - "
                               + "\n  - ".join(problems))
        out = Path(out_dir)
        if clean and out.exists():
            shutil.rmtree(out)
        out.mkdir(parents=True, exist_ok=True)
        return self.exporter_cls(self).export(scenario, out)

    # -- runtime -----------------------------------------------------------
    def create_simulation(self, artifacts: Artifacts, **options) -> Simulation:
        raise BackendError(
            f"backend '{self.name}' runs out of process; use launch_command() instead"
        )

    def launch_command(self, artifacts: Artifacts) -> list[str]:
        raise BackendError(f"backend '{self.name}' runs in process; use create_simulation() instead")

    def view(self, artifacts: Artifacts, **options) -> None:
        """Open an interactive on-screen viewer (blocks until it is closed)."""
        raise BackendError(f"backend '{self.name}' has no built-in viewer")


BACKENDS: Registry[SimBackend] = Registry("backend", "dynabarn_core.backends")


def create_backend(name: str, assets: AssetLibrary | None = None) -> SimBackend:
    return BACKENDS.create(name, assets=assets)
