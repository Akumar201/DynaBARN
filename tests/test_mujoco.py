import math
import xml.etree.ElementTree as ET

import pytest

from dynabarn_core.backends import BackendError, create_backend
from dynabarn_core.core import Pose2D, VelocityCommand
from dynabarn_core.generators import create_generator

from conftest import make_scenario, static_obstacle


def test_export_does_not_need_mujoco(tmp_path):
    s = create_generator("lanes", count=6).build(seed=1)
    art = create_backend("mujoco").export(s, tmp_path)
    root = ET.parse(art.scene).getroot()
    mocap = [b for b in root.iter("body") if b.get("mocap") == "true"]
    assert len(mocap) == 6
    assert root.find(".//body[@name='base_link']").get("pos").startswith("-9.3 0")
    assert art.scenario.is_file()


def test_check_reports_missing_assets(tmp_path):
    s = make_scenario([static_obstacle("a", 1, 1, asset="does_not_exist")])
    with pytest.raises(BackendError, match="does_not_exist"):
        create_backend("mujoco").export(s, tmp_path)


def _sim(tmp_path, scenario, **kw):
    pytest.importorskip("mujoco")
    b = create_backend("mujoco")
    return b.create_simulation(b.export(scenario, tmp_path), **kw)


def test_drives_at_commanded_speed(tmp_path):
    with _sim(tmp_path, make_scenario()) as sim:
        for _ in range(60):
            obs = sim.step(VelocityCommand(0.2, 0.0))
    assert obs.velocity.linear == pytest.approx(0.2, abs=0.01)
    assert obs.pose.x == pytest.approx(0.55, abs=0.05)
    assert abs(obs.pose.y) < 0.01


def test_lidar_sees_obstacle_at_expected_range(tmp_path):
    # cylinder r=0.25 at x=1.5 -> surface at 1.25; lidar is 0.032 m behind base_link
    s = make_scenario([static_obstacle("o", 1.5, 0.0)])
    with _sim(tmp_path, s) as sim:
        scan = sim.reset().scan
    assert len(scan.ranges) == 360
    assert scan.ranges[0] == pytest.approx(1.282, abs=0.01)
    assert math.isinf(scan.ranges[90])  # nothing within 3.5 m to the left


def test_collision_is_reported(tmp_path):
    s = make_scenario([static_obstacle("o", 0.6, 0.0)])
    with _sim(tmp_path, s) as sim:
        hit = None
        for _ in range(80):
            obs = sim.step(VelocityCommand(0.2, 0.0))
            if obs.contacts:
                hit = obs
                break
    assert hit is not None
    assert {c.other for c in hit.contacts} == {"o"}


def test_obstacles_follow_trajectories(tmp_path):
    s = create_generator("lanes", count=3, lanes=3).build(seed=0)
    with _sim(tmp_path, s) as sim:
        for _ in range(20):
            sim.step(VelocityCommand())
        t = sim.time
        for idx, o in sim._mocap:
            p = o.trajectory.pose_at(t - sim.model.opt.timestep)
            assert sim.data.mocap_pos[idx][0] == pytest.approx(p.x, abs=1e-6)


@pytest.mark.parametrize("v,w", [(1.0, 0.0), (2.0, 0.0), (0.0, 1.0), (0.5, 0.5)])
def test_jackal_tracks_commands(tmp_path, v, w):
    from dynabarn_core.core import Bounds, RobotSpec, Scenario

    s = Scenario("j", 0, "arena_20x20", Bounds(-9, 9, -8, 8), RobotSpec("jackal", Pose2D(0, 0)), Pose2D(10, 0))
    with _sim(tmp_path, s) as sim:
        for _ in range(60):
            obs = sim.step(VelocityCommand(v, w))
    assert obs.velocity.linear == pytest.approx(v, abs=0.05 + 0.05 * v)
    assert obs.velocity.angular == pytest.approx(w, abs=0.05 + 0.1 * w)


@pytest.mark.parametrize("preset", ["dynabarn_lanes", "dynabarn_random", "burger_lanes", "burger_random"])
def test_presets_run_and_robot_starts_clear(tmp_path, preset):
    from dynabarn_core.generators import generator_from_preset

    s = generator_from_preset(preset).build(seed=0)
    with _sim(tmp_path, s) as sim:
        obs = sim.reset()
        assert not obs.contacts, f"robot starts touching {obs.contacts}"
        for _ in range(20):
            obs = sim.step(VelocityCommand())
        assert not obs.contacts


def test_in_code_obstacle_runs_in_simulation(tmp_path):
    from dynabarn_core.assets import AssetLibrary, ObstacleAsset
    from dynabarn_core.core import Box

    lib = AssetLibrary(include_env=False).add(ObstacleAsset(id="crate_1m", geometry=Box((1.0, 1.0, 1.0))))
    s = make_scenario([static_obstacle("c", 1.5, 0.0, asset="crate_1m")])
    b = create_backend("mujoco", lib)
    art = b.export(s, tmp_path)
    pytest.importorskip("mujoco")
    with b.create_simulation(art) as sim:
        scan = sim.reset().scan
    # box face at x = 1.0, lidar 0.032 m behind base_link
    assert scan.ranges[0] == pytest.approx(1.032, abs=0.01)
