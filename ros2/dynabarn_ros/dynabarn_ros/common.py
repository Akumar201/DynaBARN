"""Scenario selection shared by the MuJoCo node and the Gazebo launch."""
from __future__ import annotations

from pathlib import Path

import yaml

from dynabarn_core import AssetLibrary, Scenario, create_generator
from dynabarn_core.generators import generator_from_preset


def make_scenario(scenario: str = "", preset: str = "", generator: str = "", params: str = "",
                  seed: int = 0, assets: str = "") -> tuple[Scenario, AssetLibrary]:
    """One of: ``scenario`` (a scenario file), ``preset`` or ``generator``
    (with ``params``, a YAML mapping such as ``"{lanes: 3, count: 6}"``).
    ``assets``: extra asset folders separated by ``:``."""
    lib = AssetLibrary([p for p in assets.split(":") if p])
    overrides = yaml.safe_load(params) if params.strip() else {}
    if not isinstance(overrides, dict):
        raise ValueError(f"params must be a YAML mapping, got {params!r}")
    chosen = [n for n, v in (("scenario", scenario), ("preset", preset), ("generator", generator)) if v]
    if len(chosen) > 1:
        raise ValueError(f"give only one of scenario, preset, generator (got {', '.join(chosen)})")
    if scenario:
        return Scenario.load(Path(scenario)), lib
    if preset:
        return generator_from_preset(preset, lib, **overrides).build(seed=seed, name=preset), lib
    gen = create_generator(generator or "lanes", lib, **overrides)
    return gen.build(seed=seed), lib
