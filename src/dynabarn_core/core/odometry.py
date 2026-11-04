"""Odometry estimation shared by every backend.

A backend feeds the robot's true pose and velocity (and the wheel angles,
for wheel odometry); this turns them into what the robot's odometry would
report, in the odometry frame (origin = the robot's start pose):

* ``ground_truth``: the exact pose relative to the start, no drift;
* ``wheels``: integrated wheel rotation with the differential-drive model,
  so it drifts when wheels slip (e.g. skid-steer turns), with optional
  relative noise.

Keeping this here (instead of in each simulator) makes MuJoCo and Gazebo
report identical odometry for identical motion.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .geometry import Pose2D
from .messages import Odometry, Twist2D

if TYPE_CHECKING:  # pragma: no cover
    import numpy as np

    from ..assets import DiffDriveSpec, OdometrySpec

_EPS = 1e-9


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class OdometryModel:
    def __init__(self, spec: "OdometrySpec", drive: "DiffDriveSpec") -> None:
        self.spec = spec
        self.drive = drive
        self.reset()

    def reset(self) -> None:
        self.start: Pose2D | None = None
        self.prev: tuple[float, float, float] | None = None  # time, left angle, right angle
        self.x = self.y = self.yaw = 0.0

    def update(self, t: float, pose: Pose2D, velocity: Twist2D,
               wheels: tuple[float, float] | None = None, rng: "np.random.Generator | None" = None) -> Odometry:
        """``pose``: true world pose; ``velocity``: true velocity in the robot
        frame; ``wheels``: (left, right) wheel angles in rad (mean per side)."""
        s = self.spec
        if self.start is None:
            self.start = pose
        if s.source == "ground_truth":
            st = self.start
            dx, dy = pose.x - st.x, pose.y - st.y
            c, sn = math.cos(st.yaw), math.sin(st.yaw)
            rel = Pose2D(c * dx + sn * dy, -sn * dx + c * dy, wrap_angle(pose.yaw - st.yaw))
            return Odometry(rel, velocity, "ground_truth")

        if wheels is None:
            raise ValueError("wheel odometry needs the wheel angles")
        left, right = wheels
        if self.prev is None:
            self.prev = (t, left, right)
            return Odometry(Pose2D(0.0, 0.0, 0.0), Twist2D(0.0, 0.0), "wheels")
        t0, l0, r0 = self.prev
        self.prev = (t, left, right)
        dl, dr = (left - l0) * self.drive.wheel_radius, (right - r0) * self.drive.wheel_radius
        ds, dth = (dl + dr) / 2.0, (dr - dl) / self.drive.wheel_separation
        if s.linear_noise_std > 0 and rng is not None:
            ds += rng.normal(0.0, s.linear_noise_std * abs(ds))
        if s.angular_noise_std > 0 and rng is not None:
            dth += rng.normal(0.0, s.angular_noise_std * (abs(dth) + abs(ds)))
        mid = self.yaw + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = wrap_angle(self.yaw + dth)
        dt = max(t - t0, _EPS)
        return Odometry(Pose2D(self.x, self.y, self.yaw), Twist2D(ds / dt, dth / dt), "wheels")


def body_velocity(yaw: float, vx_world: float, vy_world: float, wz: float) -> Twist2D:
    """World-frame planar velocity -> forward speed and turn rate in the robot frame."""
    return Twist2D(math.cos(yaw) * vx_world + math.sin(yaw) * vy_world, wz)
