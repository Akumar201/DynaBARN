"""World importers (Adapter): turn worlds made for other tools into
dynabarn ``EnvironmentAsset``s, which every backend can then build.

Importers keep geometry and metadata only. Simulator-specific parts (e.g.
Gazebo system plugins) are dropped: each backend adds what its simulator
needs when it exports a scenario.
"""
from __future__ import annotations

import shutil
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import ClassVar

import yaml

from ..assets import EnvironmentAsset
from ..core.registry import Registry


class WorldImportError(ValueError):
    pass


@dataclass
class ImportResult:
    environment: EnvironmentAsset
    warnings: list[str] = field(default_factory=list)
    # files copied next to asset.yaml when written (e.g. the source's LICENSE)
    extra_files: dict[str, Path] = field(default_factory=dict)
    # shapes of non-static models, by model name, at their pose in the file
    dynamic_models: dict[str, list] = field(default_factory=dict)


class WorldImporter(ABC):
    registry_name: ClassVar[str]

    @classmethod
    @abstractmethod
    def can_import(cls, path: Path) -> bool:
        """Whether ``path`` looks like something this importer reads."""

    @abstractmethod
    def load(self, path: Path, asset_id: str | None = None) -> ImportResult:
        ...


WORLD_IMPORTERS: Registry[WorldImporter] = Registry("world importer", "dynabarn_core.world_importers")


def detect_importer(path: Path) -> str:
    # most specific first
    for name in sorted(WORLD_IMPORTERS.names(), key=lambda n: n == "sdf"):
        if WORLD_IMPORTERS.get(name).can_import(path):
            return name
    raise WorldImportError(f"no importer recognises {path}. Importers: {', '.join(WORLD_IMPORTERS.names())}")


def import_world(path: str | Path, importer: str | None = None, asset_id: str | None = None) -> ImportResult:
    """Load one world. The importer is picked automatically unless given."""
    p = Path(path)
    if not p.exists():
        raise WorldImportError(f"{p} does not exist")
    return WORLD_IMPORTERS.create(importer or detect_importer(p)).load(p, asset_id)


def write_environment(result: ImportResult, out_dir: str | Path) -> Path:
    """Write ``<out_dir>/<asset id>/asset.yaml`` (plus extra files) and return
    the asset folder. The folder can then be used with ``--assets out_dir``."""
    env = result.environment
    folder = Path(out_dir) / env.id
    folder.mkdir(parents=True, exist_ok=True)
    data = env.to_manifest()
    header = f"# Imported by dynabarn-core. Source: {env.metadata.get('source', {}).get('path', '?')}\n"
    (folder / "asset.yaml").write_text(header + yaml.safe_dump(data, sort_keys=False, default_flow_style=None, width=120))
    for name, src in result.extra_files.items():
        shutil.copyfile(src, folder / name)
    result.environment = replace(env, root=folder)
    return folder
