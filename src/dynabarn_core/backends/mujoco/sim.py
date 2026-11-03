"""In-process MuJoCo runtime implementing the generic ``Simulation`` API."""
from __future__ import annotations

import dataclasses
import math
import threading

import numpy as np

from ...assets import RobotAsset
from ...core.geometry import Pose2D, yaw_to_quat_wxyz
from ...core.messages import (
    Contact,
    Observation,
    Pose3D,
    SensorInfo,
    SensorReading,
    Twist2D,
)
from ...core.scenario import Scenario
from ..base import Artifacts, BackendError, Simulation
from .exporter import OBSTACLE_PREFIX, STATIC_PREFIX

try:
    import mujoco
except ImportError as e:  # pragma: no cover - depends on the environment
    raise BackendError("the MuJoCo backend needs `pip install mujoco`") from e

from .sensors import MjSensor, make_sensor  # noqa: E402  (needs mujoco)


class MujocoSimulation(Simulation):
    def __init__(
        self,
        artifacts: Artifacts,
        robot: RobotAsset,
        control_hz: float = 20.0,
        seed: int = 0,
        enable: tuple[str, ...] | list[str] = (),
        disable: tuple[str, ...] | list[str] = (),
        render_cameras: bool | None = None,
    ) -> None:
        super().__init__()
        self.scenario = Scenario.load(artifacts.scenario)
        self.robot = robot
        self.rep = robot.require("mujoco")
        self.model = mujoco.MjModel.from_xml_path(str(artifacts.scene))
        self.data = mujoco.MjData(self.model)
        self.dt = 1.0 / control_hz
        self.rng = np.random.default_rng(seed)
        self._seed = seed
        m = self.model

        self.root = self.mj_id(mujoco.mjtObj.mjOBJ_BODY, self.rep.binding("root_body", robot.base_frame))
        self._root = self.root  # kept for the viewer
        # A side may have several driven wheels (skid steer): "front_left rear_left"
        self.act_left = [self.mj_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                         for n in self.rep.binding("left_actuator").replace(",", " ").split()]
        self.act_right = [self.mj_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                          for n in self.rep.binding("right_actuator").replace(",", " ").split()]
        self._root_qvel = m.jnt_dofadr[m.body_jntadr[self.root]]

        self._mocap = [
            (m.body_mocapid[self.mj_id(mujoco.mjtObj.mjOBJ_BODY, OBSTACLE_PREFIX + o.id)], o)
            for o in self.scenario.obstacles
        ]
        self._half_h = {o.id: float(m.body_pos[self.mj_id(mujoco.mjtObj.mjOBJ_BODY, OBSTACLE_PREFIX + o.id)][2])
                        for o in self.scenario.obstacles}
        self._robot_geoms = {g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == self.root}
        self._labels: dict[int, str] = {}
        for g in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if name.startswith(OBSTACLE_PREFIX):
                self._labels[g] = name[len(OBSTACLE_PREFIX):]
            elif name.startswith(STATIC_PREFIX):
                self._labels[g] = name.split("#", 1)[0].split("~", 1)[0]  # polygon pieces -> one object

        # sensors: specs from asset.yaml, then constructor overrides
        specs = {s.name: s for s in robot.sensors()}
        if render_cameras is not None:
            for s in robot.cameras:
                specs[s.name] = dataclasses.replace(s, enabled=render_cameras)
        for name, value in [*((n, True) for n in enable), *((n, False) for n in disable)]:
            if name not in specs:
                raise KeyError(f"no sensor '{name}'. Sensors: {', '.join(specs)}")
            specs[name] = dataclasses.replace(specs[name], enabled=value)
        self._sensors: dict[str, MjSensor] = {n: make_sensor(self, s) for n, s in specs.items()}
        self._renderer = None
        self._renderer_size = (0, 0)
        self._renderer_thread: int | None = None
        self._mounts: dict[str, Pose3D] = {}
        self._reset()

    # -- helpers used by the sensors -----------------------------------------
    def mj_id(self, objtype, name: str) -> int:
        i = mujoco.mj_name2id(self.model, objtype, name)
        if i < 0:
            raise BackendError(f"MuJoCo model has no {objtype.name[6:].lower()} named '{name}'")
        return i

    def relative_pose(self, pos, mat) -> Pose3D:
        """Pose of a world-frame (pos, 3x3 mat) relative to the robot base."""
        d = self.data
        R0 = d.xmat[self.root].reshape(3, 3)
        p = R0.T @ (np.asarray(pos) - d.xpos[self.root])
        R = R0.T @ np.asarray(mat).reshape(3, 3)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).reshape(9))
        return Pose3D(tuple(float(v) for v in p), tuple(float(v) for v in q))  # type: ignore[arg-type]

    def robot_pose(self) -> Pose2D:
        d = self.data
        x, y = d.xpos[self.root][:2]
        w, qx, qy, qz = d.xquat[self.root]
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return Pose2D(float(x), float(y), yaw)

    def robot_velocity(self) -> Twist2D:
        yaw = self.robot_pose().yaw
        qv = self.data.qvel[self._root_qvel: self._root_qvel + 6]
        return Twist2D(float(qv[0] * math.cos(yaw) + qv[1] * math.sin(yaw)), float(qv[5]))

    def contacts(self) -> tuple[Contact, ...]:
        m, d = self.model, self.data
        out = set()
        for c in d.contact[: d.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for mine, other in ((g1, g2), (g2, g1)):
                if mine in self._robot_geoms and other in self._labels:
                    part = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, mine) or mujoco.mj_id2name(
                        m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[mine]))
                    out.add(Contact(part or "robot", self._labels[other]))
        return tuple(sorted(out, key=lambda c: (c.other, c.robot_part)))

    def renderer(self, width: int, height: int):
        """Shared offscreen renderer, (re)created on first use or when a camera
        needs a larger image. Requires OpenGL (MUJOCO_GL=egl/osmesa/glfw).

        An OpenGL context only works on the thread that created it; using it
        from another thread silently returns garbage images, so that is an error.
        """
        me = threading.get_ident()
        if self._renderer is not None and self._renderer_thread != me:
            raise BackendError(
                "camera renderer was created on another thread; call "
                "release_thread_resources() on that thread before switching threads"
            )
        if self._renderer is None or width > self._renderer_size[0] or height > self._renderer_size[1]:
            if self._renderer is not None:
                self._renderer.close()
            w = max(width, self._renderer_size[0])
            h = max(height, self._renderer_size[1])
            self._renderer = mujoco.Renderer(self.model, height=h, width=w)
            self._renderer_size = (w, h)
            self._renderer_thread = me
        return self._renderer

    def release_thread_resources(self) -> None:
        if self._renderer is not None and self._renderer_thread == threading.get_ident():
            self._renderer.close()
            self._renderer = None
            self._renderer_size = (0, 0)
            self._renderer_thread = None

    # -- internals -------------------------------------------------------------
    def _update_obstacles(self) -> None:
        t = self.data.time
        for idx, o in self._mocap:
            p = o.trajectory.pose_at(t)
            self.data.mocap_pos[idx] = (p.x, p.y, self._half_h[o.id])
            self.data.mocap_quat[idx] = yaw_to_quat_wxyz(p.yaw)

    def _apply_command(self) -> None:
        d = self.robot.drive
        v = float(np.clip(self._command.linear, -d.max_linear, d.max_linear))
        w = float(np.clip(self._command.angular, -d.max_angular, d.max_angular))
        half = d.wheel_separation / 2.0
        # wheel_separation is the *effective* track (larger than the physical
        # one for skid-steer robots, whose wheels scrub sideways when turning)
        self.data.ctrl[self.act_left] = (v - w * half) / d.wheel_radius
        self.data.ctrl[self.act_right] = (v + w * half) / d.wheel_radius

    def _poll_sensors(self) -> None:
        t = float(self.data.time)
        for s in self._sensors.values():
            payload = s.poll(t)
            if payload is not None:
                self._publish(SensorReading(s.name, s.kind, s.spec.frame, t, payload))

    def _reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.rng = np.random.default_rng(self._seed)
        self._update_obstacles()
        self._apply_command()
        mujoco.mj_forward(self.model, self.data)
        self._mounts = {n: s.mount() for n, s in self._sensors.items()}
        for s in self._sensors.values():
            s.reset()
        self._poll_sensors()

    def _advance(self, seconds: float) -> None:
        n = max(1, round(seconds / self.model.opt.timestep))
        for _ in range(n):
            self._apply_command()
            self._update_obstacles()
            mujoco.mj_step(self.model, self.data)
            self._poll_sensors()

    # -- Simulation API ----------------------------------------------------------
    @property
    def time(self) -> float:
        return float(self.data.time)

    def sensors(self) -> dict[str, SensorInfo]:
        return {
            n: SensorInfo(n, s.kind, s.spec.frame, s.spec.rate_hz, s.spec.enabled,
                          self._mounts.get(n, Pose3D()), self.robot.base_frame, s.spec)
            for n, s in self._sensors.items()
        }

    def configure_sensor(self, name: str, **changes) -> SensorInfo:
        if name not in self._sensors:
            raise KeyError(f"no sensor '{name}'. Sensors: {', '.join(self._sensors)}")
        with self._lock:
            s = self._sensors[name]
            try:
                s.spec = dataclasses.replace(s.spec, **changes)
            except TypeError as e:
                raise ValueError(f"sensor '{name}': {e}") from None
            s.reconfigured()
            if changes.get("enabled"):
                s.next_t = self.time  # deliver a reading on the next physics step
            return self.sensors()[name]

    def observe(self) -> Observation:
        latest = dict(self._latest)

        def data(kind_name: str):
            r = latest.get(kind_name)
            return r.data if r is not None else None

        lidar = next((n for n, s in self._sensors.items() if s.kind == "lidar"), None)
        images = {n: r.data.data for n, r in latest.items() if r.kind == "camera"}
        return Observation(
            time=self.time,
            pose=self.robot_pose(),
            velocity=self.robot_velocity(),
            scan=data(lidar) if lidar else None,
            contacts=self.contacts(),
            images=images,
            imu=next((r.data for r in latest.values() if r.kind == "imu"), None),
            odom=next((r.data for r in latest.values() if r.kind == "odometry"), None),
            joints=next((r.data for r in latest.values() if r.kind == "joint_states"), None),
            readings=latest,
        )

    def close(self) -> None:
        self.release_thread_resources()
