"""Scenario: the simulator-independent description of one benchmark world.

A scenario references assets by id (robot, obstacles, environment) rather
than embedding meshes or simulator files, so the same scenario can be
exported to any backend and assets can be swapped without regenerating it.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .geometry import Bounds, Pose2D
from .trajectories import Trajectory, trajectory_from_dict

SCHEMA_VERSION = 1


@dataclass(frozen=True)
class ObstacleSpec:
    id: str
    asset: str
    trajectory: Trajectory

    def to_dict(self) -> dict:
        return {"id": self.id, "asset": self.asset, "trajectory": self.trajectory.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> "ObstacleSpec":
        return cls(str(d["id"]), str(d["asset"]), trajectory_from_dict(d["trajectory"]))


@dataclass(frozen=True)
class RobotSpec:
    asset: str
    start: Pose2D

    def to_dict(self) -> dict:
        return {"asset": self.asset, "start": self.start.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> "RobotSpec":
        return cls(str(d["asset"]), Pose2D.from_dict(d["start"]))


@dataclass(frozen=True)
class Scenario:
    name: str
    seed: int
    environment: str
    bounds: Bounds
    robot: RobotSpec
    goal: Pose2D
    obstacles: tuple[ObstacleSpec, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict, compare=False)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        ids = [o.id for o in self.obstacles]
        dupes = {i for i in ids if ids.count(i) > 1}
        if dupes:
            raise ValueError(f"duplicate obstacle ids: {sorted(dupes)}")

    # ---- serialisation -------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "seed": self.seed,
            "environment": self.environment,
            "bounds": self.bounds.to_dict(),
            "robot": self.robot.to_dict(),
            "goal": self.goal.to_dict(),
            "metadata": dict(self.metadata),
            "obstacles": [o.to_dict() for o in self.obstacles],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Scenario":
        version = int(d.get("schema_version", SCHEMA_VERSION))
        if version > SCHEMA_VERSION:
            raise ValueError(
                f"scenario schema v{version} is newer than this dynabarn (v{SCHEMA_VERSION})"
            )
        return cls(
            name=str(d["name"]),
            seed=int(d["seed"]),
            environment=str(d["environment"]),
            bounds=Bounds.from_dict(d["bounds"]),
            robot=RobotSpec.from_dict(d["robot"]),
            goal=Pose2D.from_dict(d["goal"]),
            obstacles=tuple(ObstacleSpec.from_dict(o) for o in d.get("obstacles", [])),
            metadata=dict(d.get("metadata") or {}),
            schema_version=version,
        )

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = self.to_dict()
        if path.suffix == ".json":
            path.write_text(json.dumps(data, indent=2))
        else:
            path.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Scenario":
        path = Path(path)
        text = path.read_text()
        data = json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)
        return cls.from_dict(data)
