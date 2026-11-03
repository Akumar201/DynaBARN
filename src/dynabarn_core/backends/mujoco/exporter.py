"""Scenario -> MJCF.

Obstacles become mocap bodies: their pose is set every physics step from
the scenario's trajectories, and they still collide with the robot. The
robot's own MJCF (from its asset) is merged into the scene and placed at
the start pose.
"""
from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from ...assets import RobotAsset
from ...core.geometry import yaw_to_quat_wxyz
from ...core.scenario import Scenario
from ...core.shapes import Box, Cylinder, ExtrudedPolygon, Shape, Sphere
from ..base import Artifacts, BackendError, Exporter

OBSTACLE_PREFIX = "obstacle:"
STATIC_PREFIX = "static:"  # walls and every other fixed environment object
GROUND = "ground"
ROBOT_DEFAULT_CLASS = "dynabarn_robot"
IMU_SITE = "dynabarn_imu"
GYRO_SENSOR = "dynabarn_gyro"
ACCEL_SENSOR = "dynabarn_accel"
# sections of the robot file merged into the scene, in MJCF order
_MERGED_SECTIONS = ("default", "asset", "worldbody", "contact", "equality", "tendon", "actuator", "sensor")


def _f(*vals: float) -> str:
    return " ".join(f"{v:.6g}" for v in vals)


def _rgba(c) -> str:
    return _f(*c)


def geom_attrs(shape: Shape) -> dict[str, str]:
    if isinstance(shape, Cylinder):
        return {"type": "cylinder", "size": _f(shape.radius, shape.length / 2)}
    if isinstance(shape, Box):
        return {"type": "box", "size": _f(*(s / 2 for s in shape.size))}
    if isinstance(shape, Sphere):
        return {"type": "sphere", "size": _f(shape.radius)}
    raise TypeError(shape)


class MujocoExporter(Exporter):
    timestep = 0.002

    def export(self, scenario: Scenario, out_dir: Path) -> Artifacts:
        env = self.assets.environment(scenario.environment)
        robot = self.assets.robot(scenario.robot.asset)

        root = ET.Element("mujoco", model=scenario.name)
        compiler = ET.SubElement(root, "compiler")
        ET.SubElement(root, "option", timestep=str(self.timestep), integrator="implicitfast")
        cams = robot.cameras
        ET.SubElement(
            ET.SubElement(root, "visual"), "global",
            offwidth=str(max([c.width for c in cams] or [640])),
            offheight=str(max([c.height for c in cams] or [480])),
        )
        sections = {name: ET.SubElement(root, name) for name in _MERGED_SECTIONS}
        asset, world = sections["asset"], sections["worldbody"]

        ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", width="512",
                      height="512", rgb1=".85 .85 .85", rgb2=".7 .7 .7")
        ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="1 1",
                      texuniform="true")
        ET.SubElement(world, "light", name="sun", pos="0 0 10", dir="0 0 -1", directional="true")
        gx, gy = env.ground_size
        cx, cy = env.ground_center
        ET.SubElement(world, "geom", name=GROUND, type="plane", pos=_f(cx, cy, 0),
                      size=_f(gx / 2, gy / 2, 0.1), material="grid")

        used: set[str] = set()
        for i, o in enumerate(env.static_objects()):
            name = STATIC_PREFIX + o.name
            if name in used:
                name = f"{name}~{i}"
            used.add(name)
            pose = {"pos": _f(o.x, o.y, o.z), "quat": _f(*yaw_to_quat_wxyz(o.yaw))}
            if isinstance(o.shape, ExtrudedPolygon):
                # MuJoCo collides meshes as convex hulls, so a concave outline is
                # split into convex prisms (one per triangle of the polygon).
                h = o.shape.height
                for k, tri in enumerate(o.shape.convex_pieces()):
                    mesh = f"{name}#{k}"
                    verts = [v for x, y in tri for v in ((x, y, 0.0), (x, y, h))]
                    ET.SubElement(asset, "mesh", name=mesh, vertex=" ".join(_f(*v) for v in verts))
                    ET.SubElement(world, "geom", name=mesh, type="mesh", mesh=mesh, rgba=_rgba(o.color), **pose)
            else:
                ET.SubElement(world, "geom", name=name, rgba=_rgba(o.color), **pose, **geom_attrs(o.shape))

        for ob in scenario.obstacles:
            a = self.assets.obstacle(ob.asset)
            p = ob.trajectory.pose_at(0.0)
            body = ET.SubElement(
                world, "body", name=OBSTACLE_PREFIX + ob.id, mocap="true",
                pos=_f(p.x, p.y, a.geometry.half_height), quat=_f(*yaw_to_quat_wxyz(p.yaw)),
            )
            ET.SubElement(body, "geom", name=OBSTACLE_PREFIX + ob.id, rgba=_rgba(a.color),
                          **geom_attrs(a.geometry))

        if scenario.environment_obstacles:
            taken = {o.id for o in scenario.obstacles}
            for mo in env.moving_objects:
                if mo.id in taken:
                    raise BackendError(f"obstacle id '{mo.id}' is used by both the scenario and "
                                       f"environment '{env.id}'")
                if isinstance(mo.shape, ExtrudedPolygon):
                    raise BackendError(f"moving object '{mo.id}': polygon shapes cannot move in MuJoCo yet")
                p = mo.trajectory.pose_at(0.0)
                body = ET.SubElement(
                    world, "body", name=OBSTACLE_PREFIX + mo.id, mocap="true",
                    pos=_f(p.x, p.y, mo.z), quat=_f(*yaw_to_quat_wxyz(p.yaw)),
                )
                ET.SubElement(body, "geom", name=OBSTACLE_PREFIX + mo.id, rgba=_rgba(mo.color),
                              **geom_attrs(mo.shape))

        self._merge_robot(robot, scenario, compiler, sections)

        for name in _MERGED_SECTIONS:  # drop empty sections
            if len(sections[name]) == 0:
                root.remove(sections[name])
        if not compiler.attrib:
            root.remove(compiler)

        ET.indent(root)
        scene = out_dir / "scene.xml"
        ET.ElementTree(root).write(scene, encoding="unicode")
        scenario_file = scenario.save(out_dir / "scenario.yaml")
        return Artifacts(backend=self.backend.name, out_dir=out_dir, scene=scene,
                         scenario=scenario_file)

    # ------------------------------------------------------------------
    def _merge_robot(self, robot: RobotAsset, scenario: Scenario, compiler: ET.Element,
                     sections: dict[str, ET.Element]) -> None:
        rep = robot.require(self.backend.name)
        src = ET.parse(rep.path).getroot()
        base_dir = rep.path.parent

        rcomp = src.find("compiler")
        dirs = {}
        if rcomp is not None:
            for key in ("assetdir", "meshdir", "texturedir"):
                if key in rcomp.attrib:
                    dirs[key] = base_dir / rcomp.attrib[key]
            compiler.attrib.update({k: v for k, v in rcomp.attrib.items() if k not in dirs})

        # Make asset file paths absolute so the scene can live anywhere.
        for el in src.iter():
            if "file" in el.attrib:
                kind_dir = dirs.get("meshdir" if el.tag in ("mesh", "hfield", "skin") else "texturedir")
                d = kind_dir or dirs.get("assetdir") or base_dir
                el.set("file", str((d / el.attrib["file"]).resolve()))
        for el in src.iter("include"):
            raise BackendError(f"{rep.path}: <include> in robot MJCF is not supported yet")

        # Wrap the robot's global defaults in a class so they don't leak into the world.
        rdefault = src.find("default")
        if rdefault is not None:
            wrapped = ET.SubElement(sections["default"], "default", {"class": ROBOT_DEFAULT_CLASS})
            wrapped.extend(list(rdefault))
            for sec in ("actuator", "sensor", "tendon", "equality"):
                for child in (src.find(sec) if src.find(sec) is not None else []):
                    child.attrib.setdefault("class", ROBOT_DEFAULT_CLASS)

        root_name = rep.binding("root_body", robot.base_frame)
        rworld = src.find("worldbody")
        body = rworld.find(f".//body[@name='{root_name}']") if rworld is not None else None
        if body is None:
            raise BackendError(f"{rep.path}: root body '{root_name}' not found")
        if rdefault is not None:
            body.attrib.setdefault("childclass", ROBOT_DEFAULT_CLASS)
        z = float(body.get("pos", "0 0 0").split()[2])
        s = scenario.robot.start
        body.set("pos", _f(s.x, s.y, z))
        body.set("quat", _f(*yaw_to_quat_wxyz(s.yaw)))
        body.attrib.pop("euler", None)

        # IMU: MuJoCo's gyro + accelerometer at the robot's imu_site (or the base).
        if robot.imu is not None:
            site = rep.bindings.get("imu_site")
            if site is None:
                site = IMU_SITE
                ET.SubElement(body, "site", name=IMU_SITE, pos="0 0 0", size="0.005")
            elif src.find(f".//site[@name='{site}']") is None:
                raise BackendError(f"{rep.path}: imu_site '{site}' not found")
            ET.SubElement(sections["sensor"], "gyro", name=GYRO_SENSOR, site=site)
            ET.SubElement(sections["sensor"], "accelerometer", name=ACCEL_SENSOR, site=site)

        for sec in _MERGED_SECTIONS:
            if sec == "default":
                continue
            el = src.find(sec)
            if el is not None:
                sections[sec].extend(list(el))
