"""Scenario -> Gazebo world (SDF 1.9) + mover and bridge configs.

* Static environment objects become static models (polygons as convex
  ``polyline`` prisms, like the MuJoCo exporter).
* Moving obstacles become gravity-free models that the mover system
  teleports onto their trajectory before every physics step (the Gazebo
  counterpart of MuJoCo mocap bodies): they push the robot, nothing pushes them.
* The robot is the asset's ``gazebo`` SDF model plus sensors and plugins
  generated from asset.yaml, so rates, noise, ranges and mounts match MuJoCo.
* Obstacle motion and odometry run in dynabarn's own Gazebo system
  (``system.py``), configured by ``system.yaml``, with the same code as MuJoCo.
"""
from __future__ import annotations

import copy
import dataclasses
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from ... import interface
from ...assets import CameraSpec, ImuSpec, LidarSpec, ObstacleAsset, RobotAsset
from ...core.scenario import Scenario
from ...core.shapes import Box, Cylinder, ExtrudedPolygon, Shape, Sphere
from ..base import Artifacts, BackendError, Exporter

WORLD = "dynabarn"
ROBOT = "robot"
GZ_NS = "/dynabarn"           # Gazebo-side topic prefix; ROS names come from the bridge
OBSTACLE_PREFIX = "obstacle_"
STATIC_PREFIX = "static_"
SYSTEM_MODULE = "dynabarn_core.backends.gazebo.system"

# ROS message type -> Gazebo message type, for the bridge
GZ_TYPES = {
    "sensor_msgs/msg/LaserScan": "gz.msgs.LaserScan",
    "sensor_msgs/msg/Imu": "gz.msgs.IMU",
    "nav_msgs/msg/Odometry": "gz.msgs.Odometry",
    "sensor_msgs/msg/JointState": "gz.msgs.Model",
    "sensor_msgs/msg/Image": "gz.msgs.Image",
    "sensor_msgs/msg/CameraInfo": "gz.msgs.CameraInfo",
}


def _f(*vals: float) -> str:
    return " ".join(f"{v:.6g}" for v in vals)


def _sub(parent: ET.Element, tag: str, text=None, **attrs) -> ET.Element:
    el = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        el.text = text if isinstance(text, str) else _f(*text) if isinstance(text, (tuple, list)) else str(text)
    return el


def _geometry(parent: ET.Element, shape: Shape) -> None:
    g = _sub(parent, "geometry")
    if isinstance(shape, Box):
        _sub(_sub(g, "box"), "size", shape.size)
    elif isinstance(shape, Cylinder):
        c = _sub(g, "cylinder")
        _sub(c, "radius", shape.radius)
        _sub(c, "length", shape.length)
    elif isinstance(shape, Sphere):
        _sub(_sub(g, "sphere"), "radius", shape.radius)
    else:
        raise TypeError(shape)


def _material(parent: ET.Element, rgba) -> None:
    m = _sub(parent, "material")
    _sub(m, "ambient", rgba)
    _sub(m, "diffuse", rgba)


def _noise(parent: ET.Element, std: float) -> None:
    if std > 0:
        n = _sub(parent, "noise")
        _sub(n, "type", "gaussian")
        _sub(n, "mean", 0.0)
        _sub(n, "stddev", std)


def _plugin(parent: ET.Element, filename: str, name: str, **params) -> ET.Element:
    p = _sub(parent, "plugin", filename=filename, name=name)
    for k, v in params.items():
        for item in (v if isinstance(v, list) else [v]):
            _sub(p, k, item if isinstance(item, str) else _f(item) if isinstance(item, float) else str(item))
    return p


def _box_inertia(mass: float, sx: float, sy: float, sz: float) -> tuple[float, float, float]:
    return (mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12, mass * (sx * sx + sy * sy) / 12)


class GazeboExporter(Exporter):
    step_size = 0.001

    def export(self, scenario: Scenario, out_dir: Path) -> Artifacts:
        env = self.assets.environment(scenario.environment)
        robot = self.assets.robot(scenario.robot.asset)

        sdf = ET.Element("sdf", version="1.9")
        world = _sub(sdf, "world", name=WORLD)
        phys = _sub(world, "physics", name="1ms", type="ignored")
        _sub(phys, "max_step_size", self.step_size)
        _sub(phys, "real_time_factor", 1.0)
        for fn, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                         ("scene-broadcaster", "SceneBroadcaster"), ("imu", "Imu"), ("contact", "Contact")):
            _plugin(world, f"gz-sim-{fn}-system", f"gz::sim::systems::{name}")
        _plugin(world, "gz-sim-sensors-system", "gz::sim::systems::Sensors", render_engine="ogre2")

        light = _sub(world, "light", type="directional", name="sun")
        _sub(light, "cast_shadows", "true")
        _sub(light, "pose", (0, 0, 10, 0, 0, 0))
        _sub(light, "diffuse", (0.9, 0.9, 0.9, 1))
        _sub(light, "specular", (0.2, 0.2, 0.2, 1))
        _sub(light, "direction", (-0.4, 0.2, -0.9))
        scene = _sub(world, "scene")
        _sub(scene, "ambient", (0.6, 0.6, 0.6, 1))
        _sub(scene, "background", (0.7, 0.8, 0.9, 1))

        self._ground(world, env)
        self._statics(world, env)
        movers = self._obstacles(world, scenario, env)
        rep = self._robot(world, robot, scenario)

        system = {"world": WORLD, "moving_objects": movers}
        if robot.odometry.enabled:
            system["odometry"] = {
                "spec": dataclasses.asdict(robot.odometry),
                "drive": dataclasses.asdict(robot.drive_for(self.backend.name)),
                "seed": int(scenario.seed),
                "robot_model": ROBOT, "base_link": rep.binding("root_link", robot.base_frame),
                "left_joints": rep.binding("left_joints").split(),
                "right_joints": rep.binding("right_joints").split(),
                "frame": robot.odometry.frame, "child_frame": interface.odom_child_frame(robot),
                "topic": f"{GZ_NS}/{robot.odometry.name}", "tf_topic": f"{GZ_NS}/tf",
            }
        system_file = out_dir / "system.yaml"
        system_file.write_text(yaml.safe_dump(system, sort_keys=False))
        _plugin(world, "gz-sim-python-system-loader-system", "gz::sim::systems::PythonSystemLoader",
                module_name=SYSTEM_MODULE, config=str(system_file.resolve()))

        ET.indent(sdf)
        scene_file = out_dir / "world.sdf"
        scene_file.write_text('<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n")
        bridge_file = out_dir / "bridge.yaml"
        bridge_file.write_text(yaml.safe_dump(self.bridge_config(robot), sort_keys=False))
        scenario_file = scenario.save(out_dir / "scenario.yaml")
        return Artifacts(backend=self.backend.name, out_dir=out_dir, scene=scene_file, scenario=scenario_file,
                         files={"bridge": bridge_file, "system": system_file})

    # ---- world -----------------------------------------------------------------
    def _ground(self, world: ET.Element, env) -> None:
        m = _sub(world, "model", name="ground")
        _sub(m, "static", "true")
        cx, cy = env.ground_center
        _sub(m, "pose", (cx, cy, 0, 0, 0, 0))
        link = _sub(m, "link", name="link")
        for tag in ("collision", "visual"):
            el = _sub(link, tag, name=tag)
            plane = _sub(_sub(el, "geometry"), "plane")
            _sub(plane, "normal", (0, 0, 1))
            _sub(plane, "size", env.ground_size)
            if tag == "visual":
                _material(el, (0.78, 0.78, 0.78, 1))

    def _statics(self, world: ET.Element, env) -> None:
        used: set[str] = set()
        for i, o in enumerate(env.static_objects()):
            name = STATIC_PREFIX + o.name
            if name in used:
                name = f"{name}_{i}"
            used.add(name)
            m = _sub(world, "model", name=name)
            _sub(m, "static", "true")
            _sub(m, "pose", (o.x, o.y, o.z, 0, 0, o.yaw))
            link = _sub(m, "link", name="link")
            if isinstance(o.shape, ExtrudedPolygon):
                # convex prisms, as in MuJoCo: robust for every physics engine
                for k, tri in enumerate(o.shape.convex_pieces()):
                    for tag in ("collision", "visual"):
                        el = _sub(link, tag, name=f"{tag}_{k}")
                        pl = _sub(_sub(el, "geometry"), "polyline")
                        for x, y in tri:
                            _sub(pl, "point", (x, y))
                        _sub(pl, "height", o.shape.height)
                        if tag == "visual":
                            _material(el, o.color)
            else:
                for tag in ("collision", "visual"):
                    el = _sub(link, tag, name=tag)
                    _geometry(el, o.shape)
                    if tag == "visual":
                        _material(el, o.color)

    def _moving_model(self, world: ET.Element, name: str, shape: Shape, z: float, pose0, color,
                      visuals: list[ET.Element] | None = None) -> None:
        m = _sub(world, "model", name=name)
        _sub(m, "pose", (pose0.x, pose0.y, z, 0, 0, pose0.yaw))
        link = _sub(m, "link", name="link")
        _sub(link, "gravity", "false")
        inert = _sub(link, "inertial")
        mass = 1000.0  # heavy: within one step the robot barely moves it before the next teleport
        _sub(inert, "mass", mass)
        if isinstance(shape, Box):
            ixx, iyy, izz = _box_inertia(mass, *shape.size)
        else:
            r = shape.radius
            h = shape.length if isinstance(shape, Cylinder) else 2 * r
            ixx = iyy = mass * (3 * r * r + h * h) / 12
            izz = mass * r * r / 2
        i = _sub(inert, "inertia")
        for k, v in (("ixx", ixx), ("iyy", iyy), ("izz", izz), ("ixy", 0), ("ixz", 0), ("iyz", 0)):
            _sub(i, k, v)
        col = _sub(link, "collision", name="collision")
        _geometry(col, shape)
        if visuals:
            for v in visuals:
                link.append(copy.deepcopy(v))
        else:
            vis = _sub(link, "visual", name="visual")
            _geometry(vis, shape)
            _material(vis, color)

    def _obstacles(self, world: ET.Element, scenario: Scenario, env) -> list[dict]:
        movers: list[dict] = []
        looks: dict[str, list[ET.Element]] = {}
        for ob in scenario.obstacles:
            a: ObstacleAsset = self.assets.obstacle(ob.asset)
            if isinstance(a.geometry, ExtrudedPolygon):
                raise BackendError(f"obstacle '{ob.id}': polygon obstacles cannot move")
            visuals = None
            rep = a.representation(self.backend.name) or a.representation("mujoco")
            if rep is not None and rep.format == "mjcf_fragment":
                if a.id not in looks:
                    looks[a.id] = _visuals_from_mjcf_fragment(rep.path)
                visuals = looks[a.id]
            name = OBSTACLE_PREFIX + ob.id
            self._moving_model(world, name, a.geometry, a.geometry.half_height, ob.trajectory.pose_at(0.0),
                               a.color, visuals)
            movers.append({"model": name, "z": a.geometry.half_height, "trajectory": ob.trajectory.to_dict()})
        if scenario.environment_obstacles:
            taken = {o.id for o in scenario.obstacles}
            for mo in env.moving_objects:
                if mo.id in taken:
                    raise BackendError(f"obstacle id '{mo.id}' is used by both the scenario and environment '{env.id}'")
                if isinstance(mo.shape, ExtrudedPolygon):
                    raise BackendError(f"moving object '{mo.id}': polygon shapes cannot move")
                name = OBSTACLE_PREFIX + mo.id
                self._moving_model(world, name, mo.shape, mo.z, mo.trajectory.pose_at(0.0), mo.color)
                movers.append({"model": name, "z": mo.z, "trajectory": mo.trajectory.to_dict()})
        return movers

    # ---- robot -----------------------------------------------------------------
    def _robot(self, world: ET.Element, robot: RobotAsset, scenario: Scenario):
        rep = robot.require(self.backend.name)
        src = ET.parse(rep.path).getroot()
        model = src.find("model") if src.tag == "sdf" else src
        if model is None or model.tag != "model":
            raise BackendError(f"{rep.path}: expected an SDF file with one <model>")
        model = copy.deepcopy(model)
        model.set("name", ROBOT)
        base_dir = rep.path.parent
        for uri in model.iter("uri"):  # relative mesh paths -> absolute, so the world can live anywhere
            if uri.text and "://" not in uri.text and not uri.text.startswith("/"):
                uri.text = str((base_dir / uri.text).resolve())
        for old in model.findall("pose"):
            model.remove(old)
        s = scenario.robot.start
        model.insert(0, _el("pose", _f(s.x, s.y, 0, 0, 0, s.yaw)))

        root_name = rep.binding("root_link", robot.base_frame)
        base = model.find(f"link[@name='{root_name}']")
        if base is None:
            raise BackendError(f"{rep.path}: root link '{root_name}' not found")
        self._sensors(base, robot, rep)
        self._drive(model, robot, rep)
        world.append(model)
        return rep

    def _sensors(self, base: ET.Element, robot: RobotAsset, rep) -> None:
        for spec in robot.sensors():
            if not spec.enabled:
                continue
            if isinstance(spec, LidarSpec):
                s = _sub(base, "sensor", name=spec.name, type="gpu_lidar")
                _sub(s, "pose", spec.mount_pose)
                _sub(s, "topic", f"{GZ_NS}/{spec.name}")
                _sub(s, "gz_frame_id", spec.frame)
                _sub(s, "update_rate", spec.rate_hz)
                _sub(s, "always_on", "true")
                lidar = _sub(s, "lidar")
                h = _sub(_sub(lidar, "scan"), "horizontal")
                _sub(h, "samples", spec.samples)
                _sub(h, "resolution", 1)
                _sub(h, "min_angle", spec.angle_min)
                _sub(h, "max_angle", spec.angle_max)
                r = _sub(lidar, "range")
                _sub(r, "min", spec.range_min)
                _sub(r, "max", spec.range_max)
                _sub(r, "resolution", 0.001)
                _noise(lidar, spec.noise_std)
            elif isinstance(spec, ImuSpec):
                s = _sub(base, "sensor", name=spec.name, type="imu")
                _sub(s, "pose", spec.mount_pose)
                _sub(s, "topic", f"{GZ_NS}/{spec.name}")
                _sub(s, "gz_frame_id", spec.frame)
                _sub(s, "update_rate", spec.rate_hz)
                _sub(s, "always_on", "true")
                imu = _sub(s, "imu")
                for group, std in (("angular_velocity", spec.gyro_noise_std),
                                   ("linear_acceleration", spec.accel_noise_std)):
                    if std > 0:
                        g = _sub(imu, group)
                        for axis in "xyz":
                            _noise(_sub(g, axis), std)
            elif isinstance(spec, CameraSpec):
                s = _sub(base, "sensor", name=spec.name, type="depth_camera" if spec.depth else "camera")
                _sub(s, "pose", spec.mount_pose)
                _sub(s, "topic", f"{GZ_NS}/{spec.name}/image_raw")
                _sub(s, "gz_frame_id", spec.optical_frame)
                _sub(s, "update_rate", spec.rate_hz)
                _sub(s, "always_on", "true")
                cam = _sub(s, "camera")
                _sub(cam, "horizontal_fov", math.radians(spec.fov_deg))
                img = _sub(cam, "image")
                _sub(img, "width", spec.width)
                _sub(img, "height", spec.height)
                _sub(img, "format", "R_FLOAT32" if spec.depth else "R8G8B8")
                clip = _sub(cam, "clip")
                _sub(clip, "near", 0.05)
                _sub(clip, "far", 50.0)
                _noise(cam, spec.noise_std / (1.0 if spec.depth else 255.0))
            elif spec.kind == "contact":
                coll = rep.bindings.get("contact_collision")
                if coll is None:
                    continue
                s = _sub(base, "sensor", name=spec.name, type="contact")
                _sub(s, "topic", f"{GZ_NS}/{spec.name}")
                _sub(s, "update_rate", spec.rate_hz)
                _sub(_sub(s, "contact"), "collision", coll)

    def _drive(self, model: ET.Element, robot: RobotAsset, rep) -> None:
        # Odometry itself comes from dynabarn's system (same model as MuJoCo);
        # DiffDrive only turns cmd_vel into wheel speeds, its own odometry goes unused.
        d, gt = robot.drive_for(self.backend.name), robot.ground_truth
        base = robot.base_frame
        _plugin(model, "gz-sim-diff-drive-system", "gz::sim::systems::DiffDrive",
                left_joint=rep.binding("left_joints").split(), right_joint=rep.binding("right_joints").split(),
                wheel_separation=d.wheel_separation, wheel_radius=d.wheel_radius,
                max_linear_velocity=d.max_linear, min_linear_velocity=-d.max_linear,
                max_angular_velocity=d.max_angular, min_angular_velocity=-d.max_angular,
                topic=f"{GZ_NS}/{interface.CMD_VEL}",
                odom_topic=f"{GZ_NS}/_diff_drive/odom", tf_topic=f"{GZ_NS}/_diff_drive/tf",
                odom_publish_frequency=1.0)
        if gt.enabled:
            _plugin(model, "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher",
                    odom_frame=gt.frame, robot_base_frame=base, odom_publish_frequency=gt.rate_hz,
                    odom_topic=f"{GZ_NS}/{gt.name}", tf_topic=f"{GZ_NS}/_{gt.name}/tf", dimensions=3)
        js = robot.joint_states
        if js.enabled:
            _plugin(model, "gz-sim-joint-state-publisher-system", "gz::sim::systems::JointStatePublisher",
                    topic=f"{GZ_NS}/{js.name}", update_rate=js.rate_hz)

    # ---- ROS bridge ----------------------------------------------------------------
    @staticmethod
    def bridge_config(robot: RobotAsset) -> list[dict]:
        """ros_gz_bridge YAML: the same topics as the MuJoCo node (see interface.py)."""
        out = [
            {"ros_topic_name": interface.CLOCK, "gz_topic_name": f"/world/{WORLD}/clock",
             "ros_type_name": "rosgraph_msgs/msg/Clock", "gz_type_name": "gz.msgs.Clock", "direction": "GZ_TO_ROS"},
            {"ros_topic_name": interface.CMD_VEL, "gz_topic_name": f"{GZ_NS}/{interface.CMD_VEL}",
             "ros_type_name": "geometry_msgs/msg/Twist", "gz_type_name": "gz.msgs.Twist", "direction": "ROS_TO_GZ"},
            {"ros_topic_name": interface.TF, "gz_topic_name": f"{GZ_NS}/tf",
             "ros_type_name": "tf2_msgs/msg/TFMessage", "gz_type_name": "gz.msgs.Pose_V", "direction": "GZ_TO_ROS"},
        ]
        for t in interface.sensor_topics(robot):
            if t.ros_type.endswith("CameraInfo"):
                gz = f"{GZ_NS}/{t.sensor}/camera_info"  # Gazebo publishes it next to the image
            elif t.ros_type.endswith("Image"):
                gz = f"{GZ_NS}/{t.sensor}/image_raw"
            else:
                gz = f"{GZ_NS}/{t.sensor}"
            out.append({"ros_topic_name": t.name, "gz_topic_name": gz, "ros_type_name": t.ros_type,
                        "gz_type_name": GZ_TYPES[t.ros_type], "direction": "GZ_TO_ROS"})
        return out


def _el(tag: str, text: str) -> ET.Element:
    e = ET.Element(tag)
    e.text = text
    return e


def _visuals_from_mjcf_fragment(path: Path) -> list[ET.Element]:
    """SDF visuals for an obstacle whose look is an MJCF fragment (e.g. the
    Microduck): its mesh geoms, with their poses, scales and colours."""
    frag = ET.parse(path).getroot()
    comp = frag.find("compiler")
    meshdir = path.parent / (comp.get("meshdir", "") if comp is not None else "")
    meshes, mats = {}, {}
    for el in (frag.find("asset") if frag.find("asset") is not None else []):
        if el.tag == "mesh":
            name = el.get("name") or Path(el.get("file", "")).stem
            meshes[name] = ((meshdir / el.get("file")).resolve(), el.get("scale", "1 1 1"))
        elif el.tag == "material":
            mats[el.get("name")] = [float(v) for v in el.get("rgba", "0.7 0.7 0.7 1").split()]
    out = []
    wb = frag.find("worldbody")
    for k, g in enumerate(wb.findall("geom") if wb is not None else []):
        if g.get("type") != "mesh" or g.get("mesh") not in meshes:
            continue
        file, scale = meshes[g.get("mesh")]
        pos = [float(v) for v in g.get("pos", "0 0 0").split()]
        w, x, y, z = (float(v) for v in g.get("quat", "1 0 0 0").split())
        v = ET.Element("visual", name=f"look_{k}")
        _sub(v, "pose", (*pos, *_quat_to_rpy(w, x, y, z)))
        mesh = _sub(_sub(v, "geometry"), "mesh")
        _sub(mesh, "uri", str(file))
        _sub(mesh, "scale", scale)
        rgba = mats.get(g.get("material"), [float(c) for c in g.get("rgba", "0.7 0.7 0.7 1").split()])
        _material(v, rgba)
        out.append(v)
    if not out:
        raise BackendError(f"{path}: no mesh geoms to draw in Gazebo")
    return out


def _quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw
