"""Obstacles with their own MuJoCo model (e.g. the Microduck)."""
import math
import os
import textwrap
from pathlib import Path

import pytest

from dynabarn_core import AssetLibrary, DynaBarn, VelocityCommand
from dynabarn_core.core import Bounds, ObstacleSpec, Pose2D, RobotSpec, Scenario, StaticTrajectory

pytest.importorskip("mujoco")


@pytest.fixture
def slim_asset(tmp_path):
    """A 1 m tall obstacle whose collision box is 0.6 m deep, but whose drawn
    model is only a thin 0.1 m slab at its back: lidar must see the slab."""
    d = tmp_path / "assets" / "slim"
    (d / "mujoco").mkdir(parents=True)
    (d / "asset.yaml").write_text(textwrap.dedent("""\
        id: slim
        kind: obstacle
        geometry: {type: box, size: [0.6, 0.6, 1.0]}
        representations:
          mujoco: {format: mjcf_fragment, path: mujoco/slim.xml}
    """))
    (d / "mujoco" / "slim.xml").write_text(textwrap.dedent("""\
        <mujoco>
          <asset><material name="red" rgba="1 0 0 1"/></asset>
          <worldbody><geom type="box" size="0.05 0.3 0.5" pos="0.25 0 0" material="red"/></worldbody>
        </mujoco>
    """))
    return AssetLibrary([tmp_path / "assets"], include_env=False)


def _scenario(asset, x=1.5, robot="tb3_burger_cam"):
    return Scenario("t", 0, "arena_20x20", Bounds(-9, 9, -8, 8), RobotSpec(robot, Pose2D(0, 0)), Pose2D(9, 0),
                    (ObstacleSpec("o", asset, StaticTrajectory(Pose2D(x, 0.0))),))


def test_custom_look_lidar_sees_model_collision_uses_box(slim_asset, tmp_path):
    db = DynaBarn(_scenario("slim"), assets=slim_asset, out_dir=tmp_path / "a")
    scan = db.get_data("lidar")
    # the slab's front face is at x = 1.5 + 0.2 = 1.7; the box's would be at 1.2
    assert scan.ranges[0] == pytest.approx(1.7 - (-0.032), abs=0.01)
    hit = set()
    for _ in range(140):  # drive into it: the box (not the slab) stops the robot
        hit |= {c.other for c in db.step(VelocityCommand(0.2, 0)).contacts}
    assert hit == {"o"}
    assert db.get_data("ground_truth").pose.x < 1.5 - 0.3 + 0.0  # stopped at the box face (x = 1.2)
    db.close()


def test_custom_look_meshes_are_shared_and_renamed(slim_asset, tmp_path):
    import xml.etree.ElementTree as ET

    from dynabarn_core import create_backend

    s = _scenario("slim")
    s = Scenario(s.name, 0, s.environment, s.bounds, s.robot, s.goal,
                 (ObstacleSpec("a", "slim", StaticTrajectory(Pose2D(2, 2))),
                  ObstacleSpec("b", "slim", StaticTrajectory(Pose2D(3, 3)))))
    art = create_backend("mujoco", slim_asset).export(s, tmp_path / "b")
    root = ET.parse(art.scene).getroot()
    assert [m.get("name") for m in root.iter("material")].count("slim/red") == 1
    for name in ("obstacle:a", "obstacle:b"):
        body = root.find(f".//body[@name='{name}']")
        box, look = body.findall("geom")
        assert box.get("group") == "3" and look.get("material") == "slim/red" and look.get("contype") == "0"


DUCK_DIR = os.environ.get("MICRODUCK_DIR")


@pytest.mark.skipif(not DUCK_DIR, reason="set MICRODUCK_DIR to a clone of pollen-robotics/microduck_rl")
def test_real_microduck(tmp_path):
    from dynabarn_core.generators import generator_from_preset
    from dynabarn_core.importers.microduck import build_microduck

    folder = build_microduck(DUCK_DIR, tmp_path / "ducks")
    assert (folder / "NOTICE.md").is_file() and (folder / "LICENSE-MODELS.md").is_file()
    lib = AssetLibrary([tmp_path / "ducks"], include_env=False)
    duck = lib.obstacle("microduck")
    assert 0.25 < duck.geometry.size[2] < 0.3                      # ~27 cm tall
    s = generator_from_preset("burger_lanes", assets=lib, obstacle_asset="microduck").build(seed=1)
    db = DynaBarn(s, assets=lib, out_dir=tmp_path / "sim")
    for _ in range(40):
        obs = db.step()
    assert not obs.contacts
    db.close()
