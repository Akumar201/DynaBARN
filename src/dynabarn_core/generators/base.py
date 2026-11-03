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
    # what to put in the world (asset ids)
    obstacle_asset: str = "cylinder_red"
    robot_asset: str = "jackal"
    environment: str = "arena_20x20"
    start_x: float = -9.3
    start_y: float = 0.0
    start_yaw: float = 0.0
    # start at one of the environment's own start poses instead (e.g. "S1")
    spawn: str | None = None
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

    @property
    def bounds(self) -> Bounds:
        return Bounds(self.x_min, self.x_max, self.y_min, self.y_max)


class ClearanceError(ValueError):
    pass


class ClearanceChecker:
    """Accepts obstacle trajectories one at a time, rejecting any that would
    come closer than ``min_distance`` (centre to centre) to one already
    accepted within the time horizon."""

    def __init__(self, min_distance: float, horizon: float, dt: float = 0.1) -> None:
        self.min_distance = min_distance
        self.times = np.arange(0.0, horizon + 1e-9, dt)
        self._tracks: list[np.ndarray] = []

    def closest(self, traj: Trajectory) -> tuple[float, int]:
        """(closest distance, index of the obstacle it is closest to)."""
        if not self._tracks:
            return float("inf"), -1
        xy = traj.positions(self.times)
        dists = [float(np.min(np.linalg.norm(t - xy, axis=1))) for t in self._tracks]
        i = int(np.argmin(dists))
        return dists[i], i

    def fits(self, traj: Trajectory) -> bool:
        return self.closest(traj)[0] >= self.min_distance - 1e-9

    def add(self, traj: Trajectory) -> None:
        self._tracks.append(traj.positions(self.times))


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
        c = self.config
        if c.min_clearance is None:
            return None
        return ClearanceChecker(2 * self.obstacle_radius + c.min_clearance, c.check_horizon)

    @abstractmethod
    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        """Return one trajectory per obstacle."""

    def scenario_bounds(self) -> Bounds:
        """Region recorded in the scenario (where obstacles move)."""
        return self.config.bounds

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
