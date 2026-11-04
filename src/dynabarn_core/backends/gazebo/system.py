"""Gazebo Python system that runs dynabarn's simulator-independent logic
inside gz-sim. Loaded by Gazebo's PythonSystemLoader (see the exporter).

* Moving obstacles: before every physics step each obstacle is teleported
  to its trajectory pose at the current simulation time, exactly when and
  where MuJoCo places its mocap bodies.
* Odometry: after every step the robot's true pose (and wheel angles) go
  through ``core.odometry.OdometryModel``, the same code MuJoCo uses, and
  the result is published (odometry message + odom -> base TF) at the
  sensor's rate.

Trajectories are dynabarn's own objects (``trajectory_from_dict``), so the
paths come from exactly the same code in Gazebo and MuJoCo.
"""
from __future__ import annotations

import math

import numpy as np
import yaml

from dynabarn_core.assets import DiffDriveSpec, OdometrySpec
from dynabarn_core.core.geometry import Pose2D
from dynabarn_core.core.odometry import OdometryModel, body_velocity
from dynabarn_core.core.trajectories import trajectory_from_dict

try:  # only available inside Gazebo
    from gz.math7 import Pose3d
    from gz.msgs10.odometry_pb2 import Odometry as GzOdometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.sim8 import Joint, Link, Model, World
    from gz.transport13 import Node
except ImportError:  # pragma: no cover - importable without Gazebo
    Pose3d = GzOdometry = Pose_V = Joint = Link = Model = World = Node = None

_EPS = 1e-9


def _secs(d) -> float:
    return d.total_seconds() if hasattr(d, "total_seconds") else float(d)


def _yaw(q) -> float:
    w, x, y, z = q.w(), q.x(), q.y(), q.z()
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _publish(pub, msg) -> None:
    if hasattr(pub, "publish"):
        pub.publish(msg)
    else:
        pub.publish_raw(msg.SerializeToString(), "gz.msgs." + type(msg).__name__)


def _stamp(header, t: float, **data: str) -> None:
    sec = int(math.floor(t))
    header.stamp.sec = sec
    header.stamp.nsec = int(round((t - sec) * 1e9))
    for k, v in data.items():
        d = header.data.add()
        d.key = k
        d.value.append(v)


class DynabarnSystem:
    def __init__(self) -> None:
        self.objects: list[tuple[object, float, object]] = []  # (model, z, trajectory)
        self.pending: list[tuple[str, float, object]] = []
        self.odom: dict | None = None

    # ---- gz-sim system interface ---------------------------------------------------
    def configure(self, entity, sdf, ecm, event_mgr) -> None:
        path = sdf.get_string("config")
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        self.world = World(entity)
        self.pending = [(o["model"], float(o["z"]), trajectory_from_dict(o["trajectory"]))
                        for o in cfg.get("moving_objects", [])]
        o = cfg.get("odometry")
        if o:
            spec = OdometrySpec(**o["spec"])
            self.node = Node()
            self.odom = {
                "cfg": o, "spec": spec, "period": 1.0 / spec.rate_hz, "next_t": 0.0,
                "model": OdometryModel(spec, DiffDriveSpec(**o["drive"])),
                "rng": np.random.default_rng(int(o.get("seed", 0))),
                "pub": self.node.advertise(o["topic"], GzOdometry),
                "tf": self.node.advertise(o["tf_topic"], Pose_V),
                "link": None, "left": [], "right": [],
            }
        print(f"[dynabarn] {len(self.pending)} moving obstacle(s), odometry: "
              f"{self.odom['spec'].source if self.odom else 'off'}  ({path})", flush=True)

    def pre_update(self, info, ecm) -> None:
        if self.pending:
            self._resolve_movers(ecm)
        if self.odom is not None and self.odom["link"] is None:
            self._resolve_robot(ecm)
        if info.paused:
            return
        t = _secs(info.sim_time)
        for model, z, traj in self.objects:  # pose at the start of the step, like MuJoCo mocap
            p = traj.pose_at(t)
            model.set_world_pose_cmd(ecm, Pose3d(p.x, p.y, z, 0.0, 0.0, p.yaw))

    def post_update(self, info, ecm) -> None:
        o = self.odom
        if o is None or o["link"] is None or info.paused:
            return
        t = _secs(info.sim_time)
        if t + _EPS < o["next_t"]:
            return
        o["next_t"] += o["period"]
        if o["next_t"] <= t + _EPS:
            o["next_t"] = t + o["period"]
        link = o["link"]
        pose = link.world_pose(ecm)
        lin = link.world_linear_velocity(ecm)
        ang = link.world_angular_velocity(ecm)
        if pose is None or lin is None or ang is None:
            return
        yaw = _yaw(pose.rot())
        p2 = Pose2D(pose.pos().x(), pose.pos().y(), yaw)
        vel = body_velocity(yaw, lin.x(), lin.y(), ang.z())
        wheels = None
        if o["spec"].source == "wheels":
            wheels = (float(np.mean([j.position(ecm)[0] for j in o["left"]])),
                      float(np.mean([j.position(ecm)[0] for j in o["right"]])))
        odom = o["model"].update(t, p2, vel, wheels, o["rng"])
        self._send(t, odom)

    # ---- helpers ------------------------------------------------------------------------
    def _resolve_movers(self, ecm) -> None:
        missing = []
        for name, z, traj in self.pending:
            ent = self.world.model_by_name(ecm, name)
            if ent:
                self.objects.append((Model(ent), z, traj))
            else:
                missing.append((name, z, traj))
        self.pending = missing

    def _resolve_robot(self, ecm) -> None:
        o, cfg = self.odom, self.odom["cfg"]
        ent = self.world.model_by_name(ecm, cfg["robot_model"])
        if not ent:
            return
        model = Model(ent)
        link_ent = model.link_by_name(ecm, cfg["base_link"])
        if not link_ent:
            return
        link = Link(link_ent)
        link.enable_velocity_checks(ecm, True)
        for side in ("left", "right"):
            o[side] = []
            for name in cfg[f"{side}_joints"]:
                j = Joint(model.joint_by_name(ecm, name))
                j.enable_position_check(ecm, True)
                o[side].append(j)
        o["link"] = link

    def _send(self, t: float, odom) -> None:
        o, cfg = self.odom, self.odom["cfg"]
        frame, child = cfg["frame"], cfg["child_frame"]
        p, v = odom.pose, odom.velocity
        qz, qw = math.sin(p.yaw / 2), math.cos(p.yaw / 2)

        msg = GzOdometry()
        _stamp(msg.header, t, frame_id=frame, child_frame_id=child)
        msg.pose.position.x, msg.pose.position.y = p.x, p.y
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        msg.twist.linear.x = v.linear
        msg.twist.angular.z = v.angular
        _publish(o["pub"], msg)

        tf = Pose_V()
        pose = tf.pose.add()
        pose.name = child
        _stamp(pose.header, t, frame_id=frame, child_frame_id=child)
        pose.position.x, pose.position.y = p.x, p.y
        pose.orientation.z, pose.orientation.w = qz, qw
        _publish(o["tf"], tf)


def get_system():
    return DynabarnSystem()
