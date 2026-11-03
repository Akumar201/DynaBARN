"""Sensor streaming: rates, values, noise, runtime configuration, DynaBarn."""
import math
import time
from collections import Counter

import numpy as np
import pytest

pytest.importorskip("mujoco")

from dynabarn_core import DynaBarn, VelocityCommand  # noqa: E402
from dynabarn_core.core import Bounds, ObstacleSpec, Pose2D, RobotSpec, Scenario, StaticTrajectory  # noqa: E402

ROBOTS = ["tb3_burger_cam", "jackal"]


def scen(robot="tb3_burger_cam", obstacles=()):
    return Scenario("t", 0, "arena_20x20", Bounds(-9, 9, -8, 8), RobotSpec(robot, Pose2D(0, 0)),
                    Pose2D(10, 0), tuple(obstacles))


@pytest.fixture
def make_db(tmp_path):
    made = []

    def make(scenario=None, **kw):
        db = DynaBarn(scenario or scen(), out_dir=tmp_path / f"db{len(made)}", **kw)
        made.append(db)
        return db

    yield make
    for db in made:
        db.close()


@pytest.mark.parametrize("robot", ROBOTS)
def test_each_sensor_fires_at_its_rate(make_db, robot):
    db = make_db(scen(robot))
    counts = Counter()
    db.subscribe("*", lambda r: counts.update([r.name]))
    for _ in range(20):  # 1 s of simulation
        db.step()
    for name, info in db.sensors().items():
        expected = info.rate_hz if info.enabled else 0
        assert counts[name] == pytest.approx(expected, abs=1), name


def test_disabled_sensors_and_constructor_overrides(make_db):
    db = make_db(disable=["imu", "lidar"], enable=["rgb"])
    assert not db.sensors()["imu"].enabled and db.sensors()["rgb"].enabled
    db.step()
    assert db.get("imu") is None and db.get("lidar") is None
    assert db.get_data("rgb").data.shape == (480, 640, 3)
    with pytest.raises(KeyError, match="no sensor 'sonar'"):
        db.get("sonar")


def test_imu_gravity_and_gyro(make_db):
    db = make_db(scen("jackal"))
    for _ in range(10):
        db.step()
    imu = db.get_data("imu")
    assert imu.linear_acceleration == pytest.approx((0, 0, 9.81), abs=0.02)
    for _ in range(40):
        db.step(VelocityCommand(0.0, 1.0))
    assert db.get_data("imu").angular_velocity[2] == pytest.approx(db.get_data("ground_truth").velocity.angular, abs=0.02)


def test_odometry_ground_truth_starts_at_zero_and_wheels_drift_on_skid_steer(make_db):
    db = make_db(scen("jackal"))
    assert db.get_data("odom").pose == Pose2D(0.0, 0.0, 0.0)
    assert db.get_data("odom").source == "ground_truth"
    db.configure_sensor("odom", source="wheels")
    db.reset()
    for _ in range(200):
        db.step(VelocityCommand(0.8, 0.6))
    w, g = db.get_data("odom").pose, db.get_data("ground_truth").pose
    assert db.get_data("odom").source == "wheels"
    assert math.hypot(w.x - g.x, w.y - g.y) > 0.1, "skid-steer wheel odometry should drift"


def test_wheel_odometry_tracks_burger_straight_line(make_db):
    db = make_db()
    db.configure_sensor("odom", source="wheels")
    db.reset()
    for _ in range(100):
        db.step(VelocityCommand(0.15, 0.0))
    w, g = db.get_data("odom").pose, db.get_data("ground_truth").pose
    assert w.x == pytest.approx(g.x, abs=0.03)


@pytest.mark.parametrize("std", [0.0, 0.02])
def test_lidar_noise(make_db, std):
    db = make_db(scen(obstacles=[ObstacleSpec("o", "cylinder_red", StaticTrajectory(Pose2D(1.5, 0)))]))
    db.configure_sensor("lidar", noise_std=std, rate_hz=50)
    db.reset()
    vals = []
    for _ in range(150):
        db.step()
        vals.append(db.get_data("lidar").ranges[0])
    assert np.mean(vals) == pytest.approx(1.282, abs=0.01)
    assert np.std(vals) == pytest.approx(std, abs=0.004)


def test_mounts_and_joint_states(make_db):
    db = make_db(scen("jackal"))
    s = db.sensors()
    assert s["lidar"].mount.position == pytest.approx((0.12, 0.0, 0.27), abs=1e-6)
    assert s["rgb"].mount.position == pytest.approx((0.215, 0.0, 0.2), abs=1e-6)
    assert s["rgb"].mount.orientation == pytest.approx((1, 0, 0, 0), abs=1e-6)  # x forward
    assert s["lidar"].parent_frame == "base_link"
    for _ in range(40):
        db.step(VelocityCommand(0.98, 0.0))
    js = db.get_data("joint_states")
    assert len(js.names) == 4
    assert js.velocities == pytest.approx((10.0,) * 4, abs=0.1)


def test_configure_sensor_rejects_unknown_field(make_db):
    with pytest.raises(ValueError, match="sensor 'lidar'"):
        make_db().configure_sensor("lidar", colour="red")


def test_background_thread_streams_imu_at_full_rate(make_db):
    db = make_db().start()
    db.set_command(0.15, 0.0)
    stamps = set()
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        r = db.get("imu")
        if r is not None:
            stamps.add(r.stamp)
        time.sleep(0.001)
    db.stop()
    assert len(stamps) >= 180, f"saw {len(stamps)} IMU samples in 1 s, expected ~200"
    assert 0.8 < db.realtime_factor < 1.2


def test_wait_for_and_step_guard(make_db):
    db = make_db().start()
    first = db.wait_for("lidar", timeout=2.0)
    nxt = db.wait_for("lidar", newer_than=first.stamp, timeout=2.0)
    assert nxt.stamp == pytest.approx(first.stamp + 0.2, abs=1e-6)  # Burger lidar: 5 Hz
    with pytest.raises(RuntimeError, match="synchronous"):
        db.step()


@pytest.mark.parametrize("timeout,moving", [(0.3, False), (None, True)])
def test_command_watchdog(make_db, timeout, moving):
    db = make_db(command_timeout=timeout).start()
    db.set_command(0.2, 0.0)
    time.sleep(1.2)
    assert (db.get_data("ground_truth").velocity.linear > 0.1) is moving


def test_camera_images_identical_in_background_and_sync_mode(make_db):
    """OpenGL contexts are per thread: images rendered by the background
    thread must match a synchronous replay pixel for pixel."""
    db = make_db(scen("jackal"), enable=["rgb", "depth"]).start()
    rgb, depth = db.wait_for("rgb", newer_than=0.3, timeout=5), db.wait_for("depth", newer_than=0.3, timeout=5)
    db.stop()
    ref = make_db(scen("jackal"), enable=["rgb", "depth"])
    got = {}
    ref.subscribe("*", lambda r: got.__setitem__((r.name, round(r.stamp, 6)), r.data.data) if r.kind == "camera" else None)
    while ref.time < max(rgb.stamp, depth.stamp) + 0.01:
        ref.step()
    assert np.array_equal(got[("rgb", round(rgb.stamp, 6))], rgb.data.data)
    assert np.array_equal(got[("depth", round(depth.stamp, 6))], depth.data.data)
    # and switching back to synchronous stepping still renders
    db.step()
    assert db.get_data("rgb").data.mean() > 10
