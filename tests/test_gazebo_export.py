"""Gazebo export (no Gazebo needed) and the simulator-independent interface."""
import math
import xml.etree.ElementTree as ET

import pytest
import yaml

from dynabarn_core import AssetLibrary, create_backend, create_generator, interface
from dynabarn_core.assets import EnvironmentAsset, StaticObject
from dynabarn_core.core import Box, ExtrudedPolygon
from dynabarn_core.core.trajectories import trajectory_from_dict

ROBOTS = ["jackal", "tb3_burger_cam"]


def _export(tmp_path, robot="jackal", **params):
    lib = AssetLibrary()
    s = create_generator("lanes", lib, robot_asset=robot, **params).build(seed=2)
    art = create_backend("gazebo", lib).export(s, tmp_path)
    return s, lib, art, ET.parse(art.scene).getroot()


@pytest.mark.parametrize("robot", ROBOTS)
def test_world_has_robot_sensors_from_asset_yaml(tmp_path, robot):
    s, lib, art, root = _export(tmp_path, robot)
    spec = lib.robot(robot)
    model = root.find("world/model[@name='robot']")
    base = model.find(f"link[@name='{spec.base_frame}']")
    sensors = {e.get("name"): e for e in base.findall("sensor")}
    lidar = sensors["lidar"]
    assert lidar.get("type") == "gpu_lidar"
    assert float(lidar.find("update_rate").text) == spec.lidar.rate_hz
    assert int(lidar.find("lidar/scan/horizontal/samples").text) == spec.lidar.samples
    assert [float(v) for v in lidar.find("pose").text.split()] == pytest.approx(spec.lidar.mount_pose)
    assert lidar.find("gz_frame_id").text == spec.lidar.frame
    assert float(sensors["imu"].find("update_rate").text) == spec.imu.rate_hz
    assert "rgb" not in sensors and "depth" not in sensors  # cameras are off by default
    # robot placed at the scenario start
    x, y, z, _, _, yaw = (float(v) for v in model.find("pose").text.split())
    assert (x, y, yaw) == pytest.approx((s.robot.start.x, s.robot.start.y, s.robot.start.yaw))
    # SDF requires unique names among a link's children
    for link in model.findall("link"):
        names = [c.get("name") for c in link if c.get("name")]
        assert len(names) == len(set(names)), link.get("name")


def test_moving_obstacles_follow_scenario_trajectories(tmp_path):
    s, _, art, root = _export(tmp_path, lanes=3, count=6)
    cfg = yaml.safe_load(art.files["system"].read_text())
    by_name = {o["model"]: o for o in cfg["moving_objects"]}
    assert len(by_name) == len(s.obstacles)
    for ob in s.obstacles:
        o = by_name[f"obstacle_{ob.id}"]
        traj = trajectory_from_dict(o["trajectory"])
        for t in (0.0, 3.3, 17.0, 101.25):
            assert traj.pose_at(t) == ob.trajectory.pose_at(t)
        m = root.find(f"world/model[@name='obstacle_{ob.id}']")
        assert m.find("link/gravity").text == "false"
        assert m.find("link/collision") is not None
    # the Python system is loaded with that config
    plugin = root.find("world/plugin[@name='gz::sim::systems::PythonSystemLoader']")
    assert plugin.find("module_name").text == "dynabarn_core.backends.gazebo.system"
    assert plugin.find("config").text == str(art.files["system"].resolve())


def test_odometry_config_uses_gazebo_drive_calibration(tmp_path):
    _, lib, art, root = _export(tmp_path, "jackal")
    cfg = yaml.safe_load(art.files["system"].read_text())["odometry"]
    gz_drive = lib.robot("jackal").drive_for("gazebo")
    assert gz_drive.wheel_separation == 0.525 != lib.robot("jackal").drive.wheel_separation
    assert cfg["drive"]["wheel_separation"] == gz_drive.wheel_separation
    dd = root.find("world/model[@name='robot']/plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(dd.find("wheel_separation").text) == gz_drive.wheel_separation
    assert [e.text for e in dd.findall("left_joint")] == ["front_left_wheel", "rear_left_wheel"]
    assert cfg["frame"] == "odom" and cfg["child_frame"] == "base_link"


@pytest.mark.parametrize("robot", ROBOTS)
def test_bridge_matches_interface(tmp_path, robot):
    _, lib, art, _ = _export(tmp_path, robot)
    bridge = yaml.safe_load(art.files["bridge"].read_text())
    ros = {b["ros_topic_name"]: b["ros_type_name"] for b in bridge}
    expected = {t.name: t.ros_type for t in interface.sensor_topics(lib.robot(robot))}
    expected.update({"/clock": "rosgraph_msgs/msg/Clock", "cmd_vel": "geometry_msgs/msg/Twist",
                     "/tf": "tf2_msgs/msg/TFMessage"})
    assert ros == expected


def test_enabled_cameras_are_exported_with_intrinsics(tmp_path):
    lib = AssetLibrary()
    robot = lib.robot("tb3_burger_cam")
    import dataclasses

    cams = tuple(dataclasses.replace(c, enabled=True) for c in robot.cameras)
    lib.add(dataclasses.replace(robot, id="burger_cams_on", cameras=cams))
    s = create_generator("empty", lib, robot_asset="burger_cams_on").build(seed=0)
    art = create_backend("gazebo", lib).export(s, tmp_path)
    base = ET.parse(art.scene).getroot().find("world/model[@name='robot']/link[@name='base_link']")
    rgb, depth = base.find("sensor[@name='rgb']"), base.find("sensor[@name='depth']")
    assert rgb.get("type") == "camera" and depth.get("type") == "depth_camera"
    assert float(rgb.find("camera/horizontal_fov").text) == pytest.approx(math.radians(60))
    assert rgb.find("gz_frame_id").text == "camera_link_optical"
    topics = {t.name for t in interface.sensor_topics(lib.robot("burger_cams_on"))}
    assert {"rgb/image_raw", "rgb/camera_info", "depth/image_raw", "depth/camera_info"} <= topics


def test_polygon_walls_become_convex_polylines(tmp_path):
    L = ((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4))
    env = EnvironmentAsset(id="lroom", ground_size=(20, 20), objects=(
        StaticObject("L", ExtrudedPolygon(L, 1.5), 1, 1, 0), StaticObject("wall", Box((10, 0.2, 1)), 0, 5, 0.5)))
    lib = AssetLibrary().add(env)
    s = create_generator("empty", lib, environment="lroom", start_x=-5, start_y=-5).build(seed=0)
    root = ET.parse(create_backend("gazebo", lib).export(s, tmp_path).scene).getroot()
    m = root.find("world/model[@name='static_L']")
    polys = m.findall("link/collision/geometry/polyline")
    assert len(polys) == len(ExtrudedPolygon(L, 1.5).convex_pieces()) >= 2
    assert all(float(p.find("height").text) == 1.5 for p in polys)


def test_static_frames_include_optical_frames():
    robot = AssetLibrary().robot("jackal")
    frames = {child: (parent, pose) for parent, child, pose in interface.static_frames(robot)}
    assert frames["base_scan"] == ("base_link", (0.12, 0.0, 0.27, 0.0, 0.0, 0.0))
    parent, pose = frames["front_camera_optical"]
    assert parent == "front_camera" and pose[3:] == pytest.approx((-math.pi / 2, 0, -math.pi / 2))


@pytest.mark.parametrize("robot", ROBOTS)
def test_asset_mounts_match_mujoco_model(tmp_path, robot):
    """asset.yaml mounts (used by Gazebo and ROS TF) must agree with the MuJoCo model."""
    pytest.importorskip("mujoco")
    import dataclasses

    lib = AssetLibrary()
    spec = lib.robot(robot)
    lib.add(dataclasses.replace(spec, id=f"{robot}_all", cameras=tuple(
        dataclasses.replace(c, enabled=True) for c in spec.cameras)))
    s = create_generator("empty", lib, robot_asset=f"{robot}_all").build(seed=0)
    backend = create_backend("mujoco", lib)
    with backend.create_simulation(backend.export(s, tmp_path)) as sim:
        for name, info in sim.sensors().items():
            if info.config.mount is None:
                continue
            x, y, z = info.config.mount[:3]
            assert info.mount.position == pytest.approx((x, y, z), abs=1e-6), name
            roll, pitch, yaw = info.config.mount[3:]
            if roll == pitch == yaw == 0:
                assert info.mount.orientation == pytest.approx((1, 0, 0, 0), abs=1e-6), name
