"""Import worlds from mertgulerx/world-collection (Apache-2.0).

    https://github.com/mertgulerx/world-collection

Each world folder holds ``<name>.sdf`` (geometry) and ``world.yaml``
(start poses, free-space outline, doors and measured difficulty values).
Geometry comes from the SDF via the generic SDF importer; the YAML adds
spawns, free space and metadata. Their moving objects are recorded in the
metadata but not imported yet.
"""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import yaml

from ..assets import Spawn
from ..core.geometry import Pose2D
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
        mo = meta.get("moving_objects")
        if mo:
            metadata["world_collection"]["moving_objects"] = mo
            result.warnings.append(
                f"{len(mo.get('objects', []))} moving objects are kept in the metadata but not simulated yet")
        # their moving objects are non-static models: already reported above
        title = meta.get("title") or path.name
        env = replace(env, description=f"world-collection {meta.get('id', '')}: {title}".strip(),
                      spawns=spawns, free_space=free_space, metadata=metadata)

        extra = {"LICENSE": path / "LICENSE"} if (path / "LICENSE").is_file() else {}
        if not spawns:
            result.warnings.append("world.yaml has no spawns")
        return ImportResult(env, result.warnings, extra)


def find_world_folders(path: Path) -> list[Path]:
    """World folders directly inside ``path`` (e.g. the repo's ``worlds/``)."""
    return sorted(p for p in path.iterdir() if WorldCollectionImporter.can_import(p))
