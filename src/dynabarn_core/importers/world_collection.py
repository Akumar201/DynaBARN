"""Import worlds from mertgulerx/world-collection (Apache-2.0).

    https://github.com/mertgulerx/world-collection

Each world folder holds ``<name>.sdf`` (geometry) and ``world.yaml``
(start poses, free-space outline, doors and measured difficulty values).
Geometry comes from the SDF via the generic SDF importer; the YAML adds
spawns, free space, moving objects and metadata.

Moving objects take their shape from the SDF model and their motion from
world.yaml, as a ``PathTrajectory``. Each one is checked on import: its
cycle length must equal ``cycle_ns`` and its pose at t = 0 must equal the
model's pose in the SDF; anything that disagrees is reported, not imported.
"""
from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import yaml

from ..assets import MovingObject, Spawn
from ..core.geometry import Pose2D
from ..core.trajectories import PathTrajectory
from .base import WORLD_IMPORTERS, ImportResult, WorldImporter, WorldImportError
from .sdf import SdfImporter, _safe_id

REPO_URL = "https://github.com/mertgulerx/world-collection"
_META_KEYS = ("id", "slug", "title", "tier", "role", "family", "parameters", "seed",
              "expected", "measured", "descriptors", "doors", "sources", "worldgen_version", "status")


@WORLD_IMPORTERS.register("world_collection")
class WorldCollectionImporter(WorldImporter):
    """A world folder from mertgulerx/world-collection (world.yaml + .sdf)."""

    @classmethod
    def can_import(cls, path: Path) -> bool:
        return path.is_dir() and (path / "world.yaml").is_file() and any(path.glob("*.sdf"))

    def load(self, path: Path, asset_id: str | None = None) -> ImportResult:
        meta_file = path / "world.yaml"
        if not meta_file.is_file():
            raise WorldImportError(f"{path}: no world.yaml")
        sdfs = sorted(path.glob("*.sdf"))
        if len(sdfs) != 1:
            raise WorldImportError(f"{path}: expected exactly one .sdf file, found {len(sdfs)}")
        meta = yaml.safe_load(meta_file.read_text()) or {}

        env_id = asset_id or "wc_" + _safe_id(path.name)
        result = SdfImporter().load(sdfs[0], env_id)
        env = result.environment

        spawns = tuple(Spawn(str(s["id"]), Pose2D(float(s["x"]), float(s["y"]), float(s.get("yaw", 0.0))))
                       for s in meta.get("spawns") or [])
        free_space = tuple(tuple((float(x), float(y)) for x, y in poly) for poly in meta.get("inclusion") or [])

        metadata = dict(env.metadata)
        metadata["source"] = {
            "importer": "world_collection", "repository": REPO_URL, "path": str(path),
            "licence": meta.get("licence", "Apache-2.0"),
            "note": "Imported from world-collection by Mert Güler (Apache-2.0); see LICENSE in this folder.",
        }
        metadata["world_collection"] = {k: meta[k] for k in _META_KEYS if k in meta}
        moving: list[MovingObject] = []
        mo = meta.get("moving_objects")
        if mo:
            metadata["world_collection"]["moving_objects"] = {k: v for k, v in mo.items() if k != "objects"}
            moving, problems = _moving_objects(mo, result.dynamic_models)
            result.warnings[:] = [w for w in result.warnings if "non-static" not in w] + problems
        title = meta.get("title") or path.name
        env = replace(env, description=f"world-collection {meta.get('id', '')}: {title}".strip(),
                      spawns=spawns, free_space=free_space, moving_objects=tuple(moving), metadata=metadata)

        extra = {"LICENSE": path / "LICENSE"} if (path / "LICENSE").is_file() else {}
        if not spawns:
            result.warnings.append("world.yaml has no spawns")
        return ImportResult(env, result.warnings, extra)


def _moving_objects(mo: dict, models: dict) -> tuple[list[MovingObject], list[str]]:
    out, problems = [], []
    turn_rate = float(mo.get("turn_rate_rad_s", 1.0))
    for o in mo.get("objects", []):
        oid = str(o["id"])
        shapes = models.get(oid, [])
        if len(shapes) != 1:
            problems.append(f"moving object {oid}: expected one shape in the SDF model, found {len(shapes)}; skipped")
            continue
        body = shapes[0]
        try:
            traj = PathTrajectory(
                waypoints=tuple((float(w[0]), float(w[1])) for w in o["waypoints"]),
                dwells=tuple(float(w[2]) if len(w) > 2 else 0.0 for w in o["waypoints"]),
                speed=float(o["speed_m_s"]), mode=o.get("mode", "loop"),
                yaw_mode=o.get("yaw_mode", "heading"), yaw=float(o.get("yaw", 0.0)),
                turn_rate=turn_rate, phase=int(o.get("phase_ns", 0)) / 1e9,
            )
        except (KeyError, ValueError) as e:
            problems.append(f"moving object {oid}: {e}; skipped")
            continue
        if "cycle_ns" in o and traj.cycle_ns != int(o["cycle_ns"]):
            problems.append(f"moving object {oid}: cycle {traj.cycle_ns} ns != world.yaml {o['cycle_ns']} ns; skipped")
            continue
        p0 = traj.pose_at(0.0)
        if (abs(p0.x - body.x) > 1e-3 or abs(p0.y - body.y) > 1e-3
                or abs(math.remainder(p0.yaw - body.yaw, 2 * math.pi)) > 1e-4):
            problems.append(f"moving object {oid}: pose at t=0 ({p0.x:.3f}, {p0.y:.3f}) differs from the SDF "
                            f"({body.x:.3f}, {body.y:.3f}); skipped")
            continue
        out.append(MovingObject(oid, body.shape, body.z, traj, body.color))
    return out, problems


def find_world_folders(path: Path) -> list[Path]:
    """World folders directly inside ``path`` (e.g. the repo's ``worlds/``)."""
    return sorted(p for p in path.iterdir() if WorldCollectionImporter.can_import(p))
