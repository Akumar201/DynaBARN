import textwrap

import pytest

from dynabarn_core.assets import AssetError, AssetLibrary
from dynabarn_core.core import Cylinder


def test_default_assets_are_found():
    lib = AssetLibrary(include_env=False)
    assert "tb3_burger_cam" in lib.ids("robot")
    assert {"cylinder_red", "box_crate"} <= set(lib.ids("obstacle"))
    robot = lib.robot("tb3_burger_cam")
    assert robot.drive.wheel_separation == pytest.approx(0.16)
    assert robot.lidar.samples == 360
    assert robot.require("mujoco").path.is_file()


def test_user_folder_overrides_default(tmp_path):
    d = tmp_path / "my_assets" / "big_cylinder"
    d.mkdir(parents=True)
    (d / "asset.yaml").write_text(textwrap.dedent("""
        id: cylinder_red
        kind: obstacle
        geometry: {type: cylinder, radius: 0.6, length: 1.0}
    """))
    lib = AssetLibrary([tmp_path / "my_assets"], include_env=False)
    assert lib.obstacle("cylinder_red").geometry == Cylinder(0.6, 1.0)


def test_env_var_search_path(tmp_path, monkeypatch):
    d = tmp_path / "pillar"
    d.mkdir()
    (d / "asset.yaml").write_text("id: pillar\nkind: obstacle\ngeometry: {type: box, size: [1, 1, 2]}\n")
    monkeypatch.setenv("DYNABARN_ASSET_PATH", str(tmp_path))
    assert AssetLibrary().obstacle("pillar").geometry.size == (1.0, 1.0, 2.0)


def test_helpful_errors():
    lib = AssetLibrary(include_env=False)
    with pytest.raises(AssetError, match="not found"):
        lib.get("nope")
    with pytest.raises(AssetError, match="is a robot, expected obstacle"):
        lib.obstacle("tb3_burger_cam")
    with pytest.raises(AssetError, match="no 'isaac' representation"):
        lib.robot("tb3_burger_cam").require("isaac")


def test_assets_defined_in_code():
    from dynabarn_core.assets import EnvironmentAsset, ObstacleAsset, Wall
    from dynabarn_core.core import Box

    lib = AssetLibrary(include_env=False)
    lib.add(
        ObstacleAsset(id="barrel", geometry=Cylinder(0.3, 0.9), color=(0.1, 0.6, 0.1, 1)),
        EnvironmentAsset(id="room", ground_size=(10, 10),
                         walls=(Wall("w", 0, 5, 0, (10, 0.2, 1)),)),
    )
    assert lib.obstacle("barrel").geometry == Cylinder(0.3, 0.9)
    assert "barrel" in lib.ids("obstacle") and "room" in lib.ids("environment")
    # code beats folders: override a built-in id
    lib.add(ObstacleAsset(id="cylinder_red", geometry=Box((1, 1, 1))))
    assert lib.obstacle("cylinder_red").geometry == Box((1.0, 1.0, 1.0))
    assert lib.source("cylinder_red") is None


def test_add_manifest_dict_and_folder(tmp_path):
    lib = AssetLibrary(include_env=False)
    a = lib.add_manifest({"id": "cone", "kind": "obstacle", "geometry": {"type": "sphere", "radius": 0.2}})
    assert lib.obstacle("cone") is a
    d = tmp_path / "pkg_assets" / "pole"
    d.mkdir(parents=True)
    (d / "asset.yaml").write_text("id: pole\nkind: obstacle\ngeometry: {type: cylinder, radius: 0.05, length: 2}\n")
    lib.add_folder(tmp_path / "pkg_assets")
    assert lib.obstacle("pole").geometry == Cylinder(0.05, 2.0)
    with pytest.raises(AssetError, match="not found"):
        AssetLibrary([tmp_path / "nope"], include_env=False)


def test_cli_assets_copy(tmp_path):
    from dynabarn_core.cli import main

    assert main(["assets", "copy", "cylinder_red", str(tmp_path), "--as", "big_red"]) == 0
    manifest = tmp_path / "big_red" / "asset.yaml"
    assert "id: big_red" in manifest.read_text()
    manifest.write_text(manifest.read_text().replace("radius: 0.25", "radius: 0.6"))
    lib = AssetLibrary([tmp_path], include_env=False)
    assert lib.obstacle("big_red").geometry.radius == 0.6
    assert lib.obstacle("cylinder_red").geometry.radius == 0.25  # original untouched
    # robots copy with their simulator files
    assert main(["assets", "copy", "jackal", str(tmp_path), "--as", "my_jackal"]) == 0
    assert AssetLibrary([tmp_path], include_env=False).robot("my_jackal").require("mujoco").path.is_file()
