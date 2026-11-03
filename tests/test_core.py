import math

import pytest

from dynabarn_core.core import (
    PolynomialTrajectory,
    Pose2D,
    Registry,
    RegistryError,
    Scenario,
    StaticTrajectory,
    WaypointTrajectory,
    trajectory_from_dict,
)

from conftest import make_scenario, static_obstacle


def test_waypoints_interpolate_and_loop():
    t = WaypointTrajectory(times=(0, 2, 4), points=((0, 0), (2, 0), (0, 0)))
    assert t.period == 4
    assert t.pose_at(1.0).x == pytest.approx(1.0)
    assert t.pose_at(1.0).yaw == pytest.approx(0.0)
    assert t.pose_at(3.0).x == pytest.approx(1.0)
    assert t.pose_at(3.0).yaw == pytest.approx(math.pi)
    assert t.pose_at(5.0).x == pytest.approx(1.0)  # wrapped


def test_waypoints_phase_shifts_time():
    t = WaypointTrajectory(times=(0, 2, 4), points=((0, 0), (2, 0), (0, 0)), phase=1.0)
    assert t.pose_at(0.0).x == pytest.approx(1.0)


def test_waypoints_from_path_uses_speed_and_dwell():
    t = WaypointTrajectory.from_path([(0, 0), (3, 4), (0, 0)], speeds=1.0, dwell=0.5)
    assert t.times == (0.0, 5.0, 10.0, 10.5)


@pytest.mark.parametrize("times", [(1, 2), (0, 0), (0, 2, 1)])
def test_waypoints_reject_bad_times(times):
    with pytest.raises(ValueError):
        WaypointTrajectory(times=times, points=[(0, 0)] * len(times))


@pytest.mark.parametrize(
    "traj",
    [
        StaticTrajectory(Pose2D(1, 2, 0.5)),
        WaypointTrajectory(times=(0, 1, 2), points=((0, 0), (1, 1), (0, 0)), phase=0.3),
        PolynomialTrajectory((0, 1, 0.5), (1, -2), duration=3.0),
    ],
)
def test_trajectory_dict_roundtrip(traj):
    again = trajectory_from_dict(traj.to_dict())
    for t in (0.0, 0.7, 2.9, 5.1):
        assert again.pose_at(t) == traj.pose_at(t)


def test_polynomial_pingpong_and_waypoint_export():
    p = PolynomialTrajectory((0, 2), (0, 0), duration=2.0)
    assert p.pose_at(0).x == pytest.approx(0)
    assert p.pose_at(2).x == pytest.approx(2)
    assert p.pose_at(4).x == pytest.approx(0)
    wp = p.as_waypoints(dt=0.05)
    for t in (0.3, 1.1, 3.3):
        assert wp.pose_at(t).x == pytest.approx(p.pose_at(t).x, abs=1e-6)


@pytest.mark.parametrize("suffix", [".yaml", ".json"])
def test_scenario_roundtrip(tmp_path, suffix):
    s = make_scenario([static_obstacle("a", 1, 2)])
    loaded = Scenario.load(s.save(tmp_path / f"s{suffix}"))
    assert loaded == s


def test_scenario_rejects_duplicate_ids_and_newer_schema():
    with pytest.raises(ValueError, match="duplicate"):
        make_scenario([static_obstacle("a", 0, 0), static_obstacle("a", 1, 1)])
    d = make_scenario().to_dict()
    d["schema_version"] = 99
    with pytest.raises(ValueError, match="newer"):
        Scenario.from_dict(d)


def test_registry_factory():
    reg: Registry = Registry("thing")

    @reg.register("a")
    class A:
        def __init__(self, x=1):
            self.x = x

    assert reg.create("a", x=3).x == 3
    assert A.registry_name == "a"
    with pytest.raises(RegistryError, match="Available: a"):
        reg.get("b")
