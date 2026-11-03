"""MuJoCo sensor implementations (one class per sensor kind).

Each sensor owns its spec (from asset.yaml, changeable at runtime), keeps
its own schedule and is polled after every physics step; when it falls due
it reads the simulator, applies its noise model and returns a payload.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

import mujoco
import numpy as np

from ...assets import (
    CameraSpec,
    ContactSpec,
    GroundTruthSpec,
    ImuSpec,
    JointStateSpec,
    LidarSpec,
    OdometrySpec,
    SensorSpec,
)
from ...core.geometry import Pose2D
from ...core.messages import (
    Contacts,
    GroundTruth,
    Image,
    Imu,
    JointStates,
    LaserScan,
    Odometry,
    Pose3D,
    Twist2D,
)
from ..base import BackendError
from .exporter import ACCEL_SENSOR, GYRO_SENSOR, IMU_SITE

if TYPE_CHECKING:
    from .sim import MujocoSimulation

_EPS = 1e-9


def _quat_from_mat(mat: np.ndarray) -> tuple[float, float, float, float]:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(mat, dtype=np.float64).reshape(9))
    return tuple(float(v) for v in q)  # type: ignore[return-value]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class MjSensor:
    kind: ClassVar[str]
    spec_cls: ClassVar[type[SensorSpec]]

    def __init__(self, sim: "MujocoSimulation", spec: SensorSpec) -> None:
        self.sim = sim
        self.spec = spec
        self.next_t = 0.0

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def period(self) -> float:
        return 1.0 / self.spec.rate_hz

    def reset(self) -> None:
        self.next_t = 0.0

    def poll(self, t: float):
        """Return a payload if the sensor is enabled and due at time ``t``."""
        if not self.spec.enabled or t + _EPS < self.next_t:
            return None
        self.next_t += self.period
        if self.next_t <= t + _EPS:  # rate faster than physics, or re-enabled: catch up
            self.next_t = t + self.period
        return self.read(t)

    def read(self, t: float):
        raise NotImplementedError

    def mount(self) -> Pose3D:
        """Sensor pose relative to the robot base frame."""
        return Pose3D()

    def reconfigured(self) -> None:
        """Called after ``configure_sensor`` changed ``self.spec``."""


class SiteMounted(MjSensor):
    site: int

    def mount(self) -> Pose3D:
        return self.sim.relative_pose(self.sim.data.site_xpos[self.site], self.sim.data.site_xmat[self.site])


class LidarSensor(SiteMounted):
    kind = "lidar"
    spec: LidarSpec

    def __init__(self, sim, spec: LidarSpec) -> None:
        super().__init__(sim, spec)
        self.site = sim.mj_id(mujoco.mjtObj.mjOBJ_SITE, sim.rep.binding("lidar_site", spec.frame))
        self.reconfigured()

    def reconfigured(self) -> None:
        n = self.spec.samples
        self.angles = np.linspace(self.spec.angle_min, self.spec.angle_max, n)
        self.local = np.stack([np.cos(self.angles), np.sin(self.angles), np.zeros(n)], 1)
        self.geom = np.zeros(n, dtype=np.int32)
        self.dist = np.zeros(n, dtype=np.float64)

    def read(self, t: float) -> LaserScan:
        s, sim = self.spec, self.sim
        pos = sim.data.site_xpos[self.site].copy()
        vec = (self.local @ sim.data.site_xmat[self.site].reshape(3, 3).T).reshape(-1)
        n = s.samples
        args = (sim.model, sim.data, pos, vec, None, 1, sim.root, self.geom, self.dist)
        try:
            mujoco.mj_multiRay(*args, None, n, s.range_max)  # MuJoCo >= 3.3 (normals argument)
        except TypeError:
            mujoco.mj_multiRay(*args, n, s.range_max)        # older MuJoCo 3.x
        r = self.dist.copy()
        hit = r >= 0
        if s.noise_std > 0:
            r[hit] += sim.rng.normal(0.0, s.noise_std, int(hit.sum()))
        valid = hit & (r >= s.range_min) & (r <= s.range_max)
        r[~valid] = math.inf
        inc = (s.angle_max - s.angle_min) / (n - 1) if n > 1 else 0.0
        return LaserScan(s.angle_min, inc, s.range_min, s.range_max, tuple(float(v) for v in r))


class CameraSensor(MjSensor):
    kind = "camera"
    spec: CameraSpec

    def __init__(self, sim, spec: CameraSpec) -> None:
        super().__init__(sim, spec)
        cam_name = sim.rep.binding(f"{spec.name}_camera")
        self.cam = sim.mj_id(mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
        self.cam_name = cam_name

    def mount(self) -> Pose3D:
        # MuJoCo cameras look along -z with y up; report the usual robot
        # convention instead: x forward, y left, z up.
        m = self.sim.data.cam_xmat[self.cam].reshape(3, 3)
        link = np.stack([-m[:, 2], -m[:, 0], m[:, 1]], axis=1)
        return self.sim.relative_pose(self.sim.data.cam_xpos[self.cam], link)

    def read(self, t: float) -> Image:
        s = self.spec
        r = self.sim.renderer(s.width, s.height)
        if s.depth:
            r.enable_depth_rendering()
        r.update_scene(self.sim.data, camera=self.cam_name)
        img = r.render().copy()
        if s.depth:
            r.disable_depth_rendering()
        if s.noise_std > 0:
            noisy = img.astype(np.float32) + self.sim.rng.normal(0.0, s.noise_std, img.shape)
            img = noisy.astype(np.float32) if s.depth else np.clip(noisy, 0, 255).astype(np.uint8)
        return Image("32FC1" if s.depth else "rgb8", img)


class ImuSensor(SiteMounted):
    kind = "imu"
    spec: ImuSpec

    def __init__(self, sim, spec: ImuSpec) -> None:
        super().__init__(sim, spec)
        m = sim.model
        self.site = sim.mj_id(mujoco.mjtObj.mjOBJ_SITE, sim.rep.bindings.get("imu_site", IMU_SITE))
        self.gyro = m.sensor_adr[sim.mj_id(mujoco.mjtObj.mjOBJ_SENSOR, GYRO_SENSOR)]
        self.accel = m.sensor_adr[sim.mj_id(mujoco.mjtObj.mjOBJ_SENSOR, ACCEL_SENSOR)]

    def read(self, t: float) -> Imu:
        s, d = self.spec, self.sim.data
        gyro = d.sensordata[self.gyro:self.gyro + 3].copy()
        acc = d.sensordata[self.accel:self.accel + 3].copy()
        if s.gyro_noise_std > 0:
            gyro += self.sim.rng.normal(0.0, s.gyro_noise_std, 3)
        if s.accel_noise_std > 0:
            acc += self.sim.rng.normal(0.0, s.accel_noise_std, 3)
        return Imu(
            orientation=_quat_from_mat(d.site_xmat[self.site]),
            angular_velocity=tuple(float(v) for v in gyro),
            linear_acceleration=tuple(float(v) for v in acc),
        )


class OdometrySensor(MjSensor):
    kind = "odometry"
    spec: OdometrySpec

    def __init__(self, sim, spec: OdometrySpec) -> None:
        super().__init__(sim, spec)
        m = sim.model

        def joints(actuators: list[int]) -> list[int]:
            return [int(m.jnt_qposadr[m.actuator_trnid[a, 0]]) for a in actuators]

        self.left_q = joints(sim.act_left)
        self.right_q = joints(sim.act_right)
        self.reset()

    def reset(self) -> None:
        super().reset()
        self.start: Pose2D | None = None
        self.prev: tuple[float, float, float] | None = None  # time, left angle, right angle
        self.x = self.y = self.yaw = 0.0

    def _wheel_angles(self) -> tuple[float, float]:
        q = self.sim.data.qpos
        return float(np.mean(q[self.left_q])), float(np.mean(q[self.right_q]))

    def read(self, t: float) -> Odometry:
        s = self.spec
        if self.start is None:
            self.start = self.sim.robot_pose()
        if s.source == "ground_truth":
            p, st = self.sim.robot_pose(), self.start
            dx, dy = p.x - st.x, p.y - st.y
            c, sn = math.cos(st.yaw), math.sin(st.yaw)
            pose = Pose2D(c * dx + sn * dy, -sn * dx + c * dy, _wrap(p.yaw - st.yaw))
            return Odometry(pose, self.sim.robot_velocity(), "ground_truth")

        # wheels: integrate wheel rotation (differential-drive model)
        drive = self.sim.robot.drive
        left, right = self._wheel_angles()
        if self.prev is None:
            self.prev = (t, left, right)
            return Odometry(Pose2D(0.0, 0.0, 0.0), Twist2D(0.0, 0.0), "wheels")
        t0, l0, r0 = self.prev
        self.prev = (t, left, right)
        dl, dr = (left - l0) * drive.wheel_radius, (right - r0) * drive.wheel_radius
        ds, dth = (dl + dr) / 2.0, (dr - dl) / drive.wheel_separation
        if s.linear_noise_std > 0:
            ds += self.sim.rng.normal(0.0, s.linear_noise_std * abs(ds))
        if s.angular_noise_std > 0:
            dth += self.sim.rng.normal(0.0, s.angular_noise_std * (abs(dth) + abs(ds)))
        mid = self.yaw + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = _wrap(self.yaw + dth)
        dt = max(t - t0, _EPS)
        return Odometry(Pose2D(self.x, self.y, self.yaw), Twist2D(ds / dt, dth / dt), "wheels")


class JointStateSensor(MjSensor):
    kind = "joint_states"
    spec: JointStateSpec

    def __init__(self, sim, spec: JointStateSpec) -> None:
        super().__init__(sim, spec)
        m = sim.model
        one_dof = {int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)}
        self.joints = [j for j in range(m.njnt)
                       if int(m.jnt_type[j]) in one_dof and m.body_rootid[m.jnt_bodyid[j]] == sim.root]
        self.names = tuple(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint_{j}" for j in self.joints)
        self.qadr = [int(m.jnt_qposadr[j]) for j in self.joints]
        self.vadr = [int(m.jnt_dofadr[j]) for j in self.joints]

    def read(self, t: float) -> JointStates:
        d = self.sim.data
        return JointStates(self.names, tuple(float(d.qpos[a]) for a in self.qadr),
                           tuple(float(d.qvel[a]) for a in self.vadr))


class ContactSensor(MjSensor):
    kind = "contact"
    spec: ContactSpec

    def read(self, t: float) -> Contacts:
        return Contacts(self.sim.contacts())


class GroundTruthSensor(MjSensor):
    kind = "ground_truth"
    spec: GroundTruthSpec

    def read(self, t: float) -> GroundTruth:
        return GroundTruth(self.sim.robot_pose(), self.sim.robot_velocity())


SENSOR_TYPES: dict[str, type[MjSensor]] = {
    cls.kind: cls
    for cls in (LidarSensor, CameraSensor, ImuSensor, OdometrySensor, JointStateSensor,
                ContactSensor, GroundTruthSensor)
}


def make_sensor(sim: "MujocoSimulation", spec: SensorSpec) -> MjSensor:
    try:
        return SENSOR_TYPES[spec.kind](sim, spec)
    except KeyError:
        raise BackendError(f"MuJoCo backend has no '{spec.kind}' sensor") from None
