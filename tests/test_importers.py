"""World importers: SDF and world-collection, polygons, round trips."""
import math
import os
import textwrap
from pathlib import Path

import pytest

from dynabarn_core import AssetLibrary, create_generator
from dynabarn_core.core import Box, Cylinder, ExtrudedPolygon
from dynabarn_core.core.geometry import polygon_area, triangulate
from dynabarn_core.importers import import_world, write_environment

L_SHAPE = ((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4))


# ---- polygons ----------------------------------------------------------------
@pytest.mark.parametrize("pts,area", [
    (((0, 0), (2, 0), (2, 3), (0, 3)), 6.0),
    (L_SHAPE, 12.0),
    (tuple(reversed(L_SHAPE)), 12.0),                         # clockwise input
    (((0, 0), (1, 0), (2, 0), (2, 2), (0, 2), (0, 2)), 4.0),  # collinear and repeated points
])
def test_triangulate_preserves_area(pts, area):
    tris = triangulate(pts)
    assert all(polygon_area(t) > 0 for t in tris)  # counter-clockwise, non-degenerate
    assert sum(polygon_area(t) for t in tris) == pytest.approx(area)


def test_triangulate_rejects_degenerate():
    with pytest.raises(ValueError):
        triangulate(((0, 0), (1, 0), (2, 0)))


# ---- SDF -------------------------------------------------------------------------
SDF = textwrap.dedent("""\
    <?xml version="1.0"?>
    <sdf version="1.9"><world name="Test World">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <include><uri>model://sun</uri></include>
      <model name="ground_plane"><static>true</static>
        <link name="l"><pose>2 3 -0.025 0 0 0</pose>
          <collision name="g"><geometry><box><size>40 30 0.05</size></box></geometry></collision></link></model>
      <model name="walls"><static>1</static><pose>10 0 0 0 0 1.5707963267948966</pose>
        <link name="l"><pose>1 0 0 0 0 0</pose>
          <collision name="wall_col"><pose>0 2 0.5 0 0 0</pose>
            <geometry><box><size>4 0.2 1</size></box></geometry></collision>
          <visual name="wall_vis"><pose>0 2 0.5 0 0 0</pose><geometry><box><size>4 0.2 1</size></box></geometry>
            <material><diffuse>0.1 0.2 0.3 1</diffuse></material></visual>
          <collision name="post_col"><pose>-3 0 0.6 0 0 0</pose>
            <geometry><cylinder><radius>0.3</radius><length>1.2</length></cylinder></geometry></collision>
          <visual name="post_vis"><geometry><cylinder><radius>0.3</radius><length>1.2</length></cylinder></geometry></visual>
        </link></model>
      <model name="house"><static>true</static>
        <link name="l"><collision name="house_col"><pose>-5 -5 0 0 0 0</pose><geometry>
          <polyline><point>0 0</point><point>4 0</point><point>4 2</point><point>2 2</point><point>2 4</point><point>0 4</point>
          <height>3</height></polyline></geometry></collision></link></model>
      <model name="cart"><pose>1 1 0 0 0 0</pose>
        <link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
      <model name="ramp"><static>true</static>
        <link name="l"><collision name="r"><pose>0 0 0 0.3 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
      <model name="statue"><static>true</static>
        <link name="l"><collision name="s"><geometry><mesh><uri>statue.dae</uri></mesh></geometry></collision></link></model>
    </world></sdf>
""")


@pytest.fixture
def sdf_file(tmp_path):
    p = tmp_path / "test.sdf"
    p.write_text(SDF)
    return p


def test_sdf_geometry_poses_and_colours(sdf_file):
    res = import_world(sdf_file)
    env = res.environment
    assert env.id == "test_world"
    by = {o.name: o for o in env.objects}
    assert set(by) == {"wall_col", "post_col", "house_col"}
    # model at (10,0) yawed 90 deg; link at +1 along model x -> (10, 1); collision at (0, 2) in link
    # frame -> rotated by 90 deg -> (-2, 0) -> world (8, 1)
    w = by["wall_col"]
    assert (w.x, w.y, w.z) == pytest.approx((8.0, 1.0, 0.5))
    assert w.yaw == pytest.approx(math.pi / 2)
    assert w.shape == Box((4.0, 0.2, 1.0)) and w.color == (0.1, 0.2, 0.3, 1.0)
    p = by["post_col"]
    assert (p.x, p.y) == pytest.approx((10.0, -2.0)) and p.shape == Cylinder(0.3, 1.2)
    h = by["house_col"]
    assert isinstance(h.shape, ExtrudedPolygon) and h.shape.height == 3.0 and (h.x, h.y) == (-5.0, -5.0)
    assert env.ground_size == (40.0, 30.0) and env.ground_center == (2.0, 3.0)


def test_sdf_reports_what_it_cannot_import(sdf_file):
    warnings = "\n".join(import_world(sdf_file).warnings)
    assert "<include>" in warnings
    assert "tilted" in warnings and "'r'" in warnings
    assert "<mesh>" in warnings
    assert "non-static" in warnings and "cart" in warnings


def test_sdf_round_trip_through_asset_folder(sdf_file, tmp_path):
    res = import_world(sdf_file, asset_id="my_world")
    folder = write_environment(res, tmp_path / "assets")
    assert folder.name == "my_world"
    loaded = AssetLibrary([tmp_path / "assets"], include_env=False).environment("my_world")
    assert loaded.objects == res.environment.objects
    assert loaded.ground_size == res.environment.ground_size


# ---- world-collection ---------------------------------------------------------------
@pytest.fixture
def wc_folder(tmp_path, sdf_file):
    d = tmp_path / "SC9_test_world"
    d.mkdir()
    (d / "SC9_test_world.sdf").write_text(SDF)
    (d / "LICENSE").write_text("Apache License 2.0 (test copy)")
    (d / "world.yaml").write_text(textwrap.dedent("""\
        id: SC9
        title: Test world
        tier: SC
        measured: {free_area_m2: 100.0}
        spawns:
        - {id: S1, x: 0.0, y: 0.0, yaw: 0.0}
        - {id: S2, x: 3.0, y: -2.0, yaw: 1.5}
        inclusion:
        - [[-10, -10], [10, -10], [10, 10], [-10, 10]]
        moving_objects: null
        licence: Apache-2.0
    """))
    return d


def test_world_collection_import(wc_folder, tmp_path):
    res = import_world(wc_folder)  # importer detected from the folder layout
    env = res.environment
    assert env.id == "wc_sc9_test_world"
    assert [s.id for s in env.spawns] == ["S1", "S2"]
    assert env.spawn("S2").yaw == 1.5
    assert len(env.free_space) == 1 and len(env.free_space[0]) == 4
    assert env.metadata["world_collection"]["measured"]["free_area_m2"] == 100.0
    assert env.metadata["source"]["licence"] == "Apache-2.0"
    folder = write_environment(res, tmp_path / "out")
    assert (folder / "LICENSE").read_text().startswith("Apache")
    again = AssetLibrary([tmp_path / "out"], include_env=False).environment(env.id)
    assert again.spawns == env.spawns and again.free_space == env.free_space


def test_cli_imports_a_folder_of_worlds(wc_folder, tmp_path):
    from dynabarn_core.cli import main

    assert main(["import-world", str(wc_folder.parent), "-o", str(tmp_path / "imported")]) == 0
    assert (tmp_path / "imported" / "wc_sc9_test_world" / "asset.yaml").is_file()


# ---- simulating an imported world --------------------------------------------------
def test_empty_generator_uses_spawns_and_world_extent(wc_folder, tmp_path):
    res = import_world(wc_folder)
    lib = AssetLibrary(include_env=False).add(res.environment)
    s = create_generator("empty", assets=lib, environment=res.environment.id, spawn="S2").build(seed=0)
    assert (s.robot.start.x, s.robot.start.y, s.robot.start.yaw) == (3.0, -2.0, 1.5)
    assert not s.obstacles
    s1 = create_generator("empty", assets=lib, environment=res.environment.id).build(seed=0)
    assert s1.robot.start.x == 0.0  # first spawn by default
    x0, x1, y0, y1 = res.environment.extent()
    assert (s.bounds.x_min, s.bounds.x_max) == (pytest.approx(x0), pytest.approx(x1))


def test_lidar_sees_imported_polygon(tmp_path):
    pytest.importorskip("mujoco")
    from dynabarn_core import DynaBarn
    from dynabarn_core.assets import EnvironmentAsset, StaticObject

    # concave L building, rotated 90 deg: world footprint x in [1,5], y in [-1,3],
    # with the open notch at x in [1,3], y in [1,3]
    env = EnvironmentAsset(id="ltest", ground_size=(40, 40), objects=(
        StaticObject("L", ExtrudedPolygon(L_SHAPE, 2.0), 5.0, -1.0, 0.0, math.pi / 2),))
    lib = AssetLibrary(include_env=False).add(env)
    s = create_generator("empty", assets=lib, environment="ltest", robot_asset="tb3_burger_cam",
                         start_x=-1.0, start_y=0.0).build(seed=0)
    db = DynaBarn(s, assets=lib, out_dir=tmp_path / "sim")
    scan = db.get_data("lidar")

    def ray(deg):
        return scan.ranges[round(math.radians(deg) / scan.angle_increment) % len(scan.ranges)]

    assert ray(0) == pytest.approx(1.0 - (-1.032), abs=0.01)                 # solid arm at x = 1
    assert math.isinf(ray(math.degrees(math.atan2(2.0, 2.032))))            # through the notch
    db.close()


# ---- optional: the real collection --------------------------------------------------
WC_DIR = os.environ.get("WORLD_COLLECTION_DIR")


@pytest.mark.skipif(not WC_DIR, reason="set WORLD_COLLECTION_DIR to <clone>/worlds to run")
def test_real_world_collection_imports_without_warnings(tmp_path):
    from dynabarn_core.importers import find_world_folders

    folders = find_world_folders(Path(WC_DIR))
    assert len(folders) >= 43
    for f in folders:
        res = import_world(f)
        env = res.environment
        assert env.spawns, f
        static_warnings = [w for w in res.warnings if "moving" not in w and "non-static" not in w]
        assert not static_warnings, (f, static_warnings)
