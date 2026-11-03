"""Presets: named generator setups stored as YAML in the project's ``presets/``.

A preset file looks like::

    description: one line shown by `dynabarn list presets`
    generator: lanes
    params: {robot_asset: tb3_burger_cam, count: 18, ...}

``name`` may be a preset name or a path to such a file. Extra keyword
arguments override the preset's params.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from ..assets import AssetLibrary
from .base import Generator, create_generator

PRESET_DIR = Path(__file__).resolve().parents[1] / "data" / "presets"


def preset_path(name: str) -> Path:
    p = Path(name)
    if p.suffix in (".yaml", ".yml") and p.is_file():
        return p
    candidate = PRESET_DIR / f"{name}.yaml"
    if candidate.is_file():
        return candidate
    raise LookupError(f"unknown preset '{name}'. Available: {', '.join(list_presets()) or 'none'}")


def list_presets() -> list[str]:
    return sorted(p.stem for p in PRESET_DIR.glob("*.yaml")) if PRESET_DIR.is_dir() else []


def load_preset(name: str) -> dict:
    data = yaml.safe_load(preset_path(name).read_text()) or {}
    if "generator" not in data:
        raise ValueError(f"preset '{name}' has no 'generator'")
    data.setdefault("params", {})
    data.setdefault("description", "")
    return data


def generator_from_preset(name: str, assets: AssetLibrary | None = None, **overrides: Any) -> Generator:
    data = load_preset(name)
    return create_generator(data["generator"], assets=assets, **{**(data["params"] or {}), **overrides})
