"""Scenario generators (Factory + Template Method).

A generator owns a ``Config`` dataclass and implements
``make_trajectories``; the base class turns those into a ``Scenario`` and
checks that obstacles never overlap. Create one by name with
``create_generator("lanes", lanes=6, count=36)``; string values (e.g. from
the CLI) are converted to the field's type.

Defaults describe the standard DynaBARN setup: a Jackal in the 20 x 20 m
arena. See ``presets/`` for other ready-made setups (e.g. TurtleBot3 scale).
"""
from __future__ import annotations

import dataclasses
import random
import typing
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, ClassVar, Sequence

import numpy as np

from ..assets import AssetLibrary
from ..core.geometry import Bounds, Pose2D
from ..core.registry import Registry
from ..core.scenario import ObstacleSpec, RobotSpec, Scenario
from ..core.trajectories import Trajectory, WaypointTrajectory


@dataclass
class GeneratorConfig:
    """Fields shared by every generator."""

    count: int = 36
    # region the obstacles move in (obstacle centres stay inside it)
    x_min: float = -8.0
    x_max: float = 8.0
    y_min: float = -8.0
    y_max: float = 8.0
    # speeds (m/s). speed_mode: constant | agent (random per obstacle) | segment (random per segment)
    speed: float = 1.0
    speed_min: float | None = None
    speed_max: float | None = None
    speed_mode: str = "constant"
    dwell: float = 0.25
    stagger_phase: bool = False
    # obstacles must keep at least this much space between their surfaces (m)
    # for the first `check_horizon` seconds. None allows them to overlap.
    min_clearance: float | None = 0.0
    check_horizon: float = 120.0
    # where obstacles may move: "bounds" (x_min..y_max above) or "environment"
    # (the environment's free-space outline, or its extent if it has none)
    area: str = "bounds"
    # keep obstacle surfaces this far from the robot's outline at its start and goal (m)
    keep_clear: float = 0.3
    # keep obstacle surfaces this far from walls and other static objects (m)
    static_clearance: float = 0.1
    # what to put in the world (asset ids)
    obstacle_asset: str = "cylinder_red"
    robot_asset: str = "jackal"
    environment: str = "arena_20x20"
    start_x: float = -9.3
    start_y: float = 0.0
    start_yaw: float = 0.0
    # start at one of the environment's own start poses instead (e.g. "S1")
    spawn: str | None = None
    # also simulate the environment's own moving objects (e.g. world-collection MO worlds)
    environment_obstacles: bool = True
    goal_x: float = 10.0
    goal_y: float = 0.0

    def __post_init__(self) -> None:
        if self.speed_mode not in ("constant", "agent", "segment"):
            raise ValueError("speed_mode must be constant, agent or segment")
        if self.speed_mode != "constant" and (self.speed_min is None or self.speed_max is None):
            raise ValueError(f"speed_mode={self.speed_mode} needs speed_min and speed_max")
        if self.count < 0:
            raise ValueError("count must be >= 0")
        if self.min_clearance is not None and self.min_clearance < 0:
            raise ValueError("min_clearance must be >= 0 (or None to allow overlaps)")
        if self.area not in ("bounds", "environment"):
            raise ValueError("area must be 'bounds' or 'environment'")
        if self.keep_clear < 0 or self.static_clearance < 0:
            raise ValueError("keep_clear and static_clearance must be >= 0")

    @property
    def bounds(self) -> Bounds:
        return Bounds(self.x_min, self.x_max, self.y_min, self.y_max)


class ClearanceError(ValueError):
    pass


class ClearanceChecker:
    """Accepts moving things one at a time and reports how close a new one
    would come to those already accepted, as the gap between their footprint
    circles (centre distance minus both radii), sampled over the horizon."""

    def __init__(self, clearance: float, horizon: float, dt: float = 0.1) -> None:
        self.clearance = clearance
        self.times = np.arange(0.0, horizon + 1e-9, dt)
        self._tracks: list[tuple[np.ndarray, float, str]] = []

    def closest(self, traj: Trajectory, radius: float) -> tuple[float, str]:
        """(smallest surface gap, label of the thing it is closest to)."""
        if not self._tracks:
            return float("inf"), ""
        xy = traj.positions(self.times)
        best = (float("inf"), "")
        for track, r, label in self._tracks:
            gap = float(np.min(np.linalg.norm(track - xy, axis=1))) - r - radius
            if gap < best[0]:
                best = (gap, label)
        return best

    def fits(self, traj: Trajectory, radius: float) -> bool:
        return self.closest(traj, radius)[0] >= self.clearance - 1e-9

    def add(self, traj: Trajectory, radius: float, label: str = "") -> None:
        self._tracks.append((traj.positions(self.times), radius, label))


class Generator(ABC):
    Config: ClassVar[type[GeneratorConfig]] = GeneratorConfig
    registry_name: ClassVar[str]

    def __init__(self, config: GeneratorConfig | None = None, assets: AssetLibrary | None = None,
                 **overrides: Any) -> None:
        if config is None:
            config = self.Config(**_coerce(self.Config, overrides))
        elif overrides:
            config = dataclasses.replace(config, **_coerce(self.Config, overrides))
        self.config = config
        self._assets = assets

    @property
    def assets(self) -> AssetLibrary:
        if self._assets is None:
            self._assets = AssetLibrary()
        return self._assets

    @property
    def obstacle_radius(self) -> float:
        return self.assets.obstacle(self.config.obstacle_asset).geometry.footprint_radius

    def clearance_checker(self) -> ClearanceChecker | None:
        """Overlap check, preloaded with the environment's own moving objects."""
        c = self.config
        if c.min_clearance is None:
            return None
        checker = ClearanceChecker(c.min_clearance, c.check_horizon)
        if c.environment_obstacles:
            for mo in self.assets.environment(c.environment).moving_objects:
                checker.add(mo.trajectory, mo.shape.footprint_radius, mo.id)
        return checker

    def region(self) -> Bounds:
        """Rectangle obstacles move in (see ``area``)."""
        c = self.config
        if c.area == "bounds":
            return c.bounds
        env = self.assets.environment(c.environment)
        if env.free_space:
            xs = [p[0] for poly in env.free_space for p in poly]
            ys = [p[1] for poly in env.free_space for p in poly]
            return Bounds(min(xs), max(xs), min(ys), max(ys))
        return Bounds(*env.extent())

    def free_space(self) -> "FreeSpace":
        """Where obstacles may go: the region, minus static objects and keep-clear
        circles around the robot's start and goal. Cached per generator."""
        if getattr(self, "_free_space", None) is None:
            from .freespace import FreeSpace

            c = self.config
            start = self.start_pose()
            # measured from the robot's outline, so a robot waiting at its start or goal is never touched
            k = self.assets.robot(c.robot_asset).footprint_radius + c.keep_clear
            keep = [(start.x, start.y, k), (c.goal_x, c.goal_y, k)]
            self._free_space = FreeSpace(self.assets.environment(c.environment), self.region(), keep)
        return self._free_space

    def obstacle_free(self, path, closed: bool = True) -> bool:
        """Whether an obstacle can travel ``path`` without touching static objects."""
        r = self.obstacle_radius + self.config.static_clearance
        return self.free_space().path_free(path, r, closed=closed)

    @abstractmethod
    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        """Return one trajectory per obstacle."""

    def scenario_bounds(self) -> Bounds:
        """Region recorded in the scenario (where obstacles move)."""
        return self.region()

    def start_pose(self) -> Pose2D:
        c = self.config
        if c.spawn:
            return self.assets.environment(c.environment).spawn(c.spawn)
        return Pose2D(c.start_x, c.start_y, c.start_yaw)

    # -- shared helpers -------------------------------------------------
    def make_waypoints(
        self, path: Sequence[tuple[float, float]], rng: random.Random, agent_speed: float | None = None,
        phase: float = 0.0,
    ) -> WaypointTrajectory:
        c = self.config
        n_seg = len(path) - 1
        if c.speed_mode == "segment":
            speeds: list[float] | float = [rng.uniform(c.speed_min, c.speed_max) for _ in range(n_seg)]
        elif c.speed_mode == "agent":
            speeds = agent_speed if agent_speed is not None else rng.uniform(c.speed_min, c.speed_max)
        else:
            speeds = c.speed
        return WaypointTrajectory.from_path(path, speeds, dwell=c.dwell, loop=True, phase=phase)

    # -- template method ------------------------------------------------
    def build(self, seed: int = 0, name: str | None = None) -> Scenario:
        c = self.config
        rng = random.Random(seed)
        trajs = self.make_trajectories(rng)
        return Scenario(
            name=name or f"{self.registry_name}_{seed}",
            seed=seed,
            environment=c.environment,
            bounds=self.scenario_bounds(),
            robot=RobotSpec(c.robot_asset, self.start_pose()),
            goal=Pose2D(c.goal_x, c.goal_y),
            obstacles=tuple(
                ObstacleSpec(f"obs_{i}", c.obstacle_asset, t) for i, t in enumerate(trajs)
            ),
            environment_obstacles=c.environment_obstacles,
            metadata={"generator": self.registry_name, "config": dataclasses.asdict(c)},
        )


GENERATORS: Registry[Generator] = Registry("generator", "dynabarn_core.generators")


def create_generator(name: str, assets: AssetLibrary | None = None, **params: Any) -> Generator:
    return GENERATORS.create(name, assets=assets, **params)


def _coerce(config_cls: type, params: dict[str, Any]) -> dict[str, Any]:
    """Convert string values to the dataclass field types; reject unknown keys."""
    hints = typing.get_type_hints(config_cls)
    fields = {f.name for f in dataclasses.fields(config_cls)}
    out: dict[str, Any] = {}
    for key, value in params.items():
        if key not in fields:
            raise TypeError(
                f"{config_cls.__name__} has no option '{key}'. Options: {', '.join(sorted(fields))}"
            )
        out[key] = _convert(value, hints[key]) if isinstance(value, str) else value
    return out


def _convert(value: str, hint: Any) -> Any:
    args = typing.get_args(hint)
    if type(None) in args:
        if value.lower() in ("none", "null", ""):
            return None
        hint = next(a for a in args if a is not type(None))
    if hint is bool:
        if value.lower() in ("1", "true", "yes", "on"):
            return True
        if value.lower() in ("0", "false", "no", "off"):
            return False
        raise ValueError(f"not a boolean: {value!r}")
    if hint in (int, float, str):
        return hint(value)
    return value
