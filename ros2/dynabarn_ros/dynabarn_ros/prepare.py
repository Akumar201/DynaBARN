"""Generate (or load) a scenario and export it for Gazebo. Used by
gazebo.launch.py; also handy on its own:

    ros2 run dynabarn_ros prepare_gazebo --preset dynabarn_lanes --seed 3 -o /tmp/world
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from dynabarn_core import create_backend
from dynabarn_core.backends import Artifacts

from .common import make_scenario


def prepare(out_dir: str | Path, **kwargs) -> tuple[Artifacts, str]:
    """Export for Gazebo; returns the artifacts and the robot asset id."""
    scenario, lib = make_scenario(**kwargs)
    art = create_backend("gazebo", lib).export(scenario, out_dir, clean=True)
    return art, scenario.robot.asset


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="")
    ap.add_argument("--preset", default="")
    ap.add_argument("--generator", default="")
    ap.add_argument("--params", default="")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--assets", default="")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--json", action="store_true", help="print the result as JSON (for launch files)")
    a = ap.parse_args()
    art, robot = prepare(a.output, scenario=a.scenario, preset=a.preset, generator=a.generator,
                         params=a.params, seed=a.seed, assets=a.assets)
    import dynabarn_core

    info = {"world": str(art.scene), "bridge": str(art.files["bridge"]), "robot": robot,
            # gz-sim's embedded Python needs this on PYTHONPATH to load dynabarn's system
            "core_path": str(Path(dynabarn_core.__file__).resolve().parents[1])}
    print(json.dumps(info) if a.json else "\n".join(f"{k}: {v}" for k, v in info.items()))
