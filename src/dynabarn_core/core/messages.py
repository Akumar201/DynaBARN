"""Simulator-independent command and sensor data types.

Every ``Simulation`` consumes ``VelocityCommand`` and produces these types.
Anything built on top (your controller, a logger, a future middleware
bridge) only depends on them, never on a particular simulator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .geometry import Pose2D

Vector3 = tuple[float, float, float]
QuaternionWXYZ = tuple[float, float, float, float]


@dataclass(frozen=True)
class VelocityCommand:
    linear: float = 0.0   # m/s, forward
    angular: float = 0.0  # rad/s, counter-clockwise


@dataclass(frozen=True)
class Twist2D:
    linear: float
    angular: float


@dataclass(frozen=True)
class Pose3D:
    position: Vector3 = (0.0, 0.0, 0.0)
    orientation: QuaternionWXYZ = (1.0, 0.0, 0.0, 0.0)


# ---- sensor payloads -----------------------------------------------------
@dataclass(frozen=True)
class LaserScan:
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple[float, ...]  # inf when nothing is hit within range_max

    @property
    def angle_max(self) -> float:
        return self.angle_min + self.angle_increment * (len(self.ranges) - 1)


@dataclass(frozen=True)
class Image:
    encoding: str  # "rgb8" (H x W x 3 uint8) or "32FC1" (H x W float32 metres)
    data: Any      # numpy array


@dataclass(frozen=True)
class Imu:
    orientation: QuaternionWXYZ
    angular_velocity: Vector3     # rad/s, sensor frame
    linear_acceleration: Vector3  # m/s^2, sensor frame, includes gravity (+9.81 z at rest)


@dataclass(frozen=True)
class Odometry:
    """Pose in the odometry frame, which starts at the robot's start pose."""

    pose: Pose2D
    velocity: Twist2D  # in the robot frame
    source: str        # "ground_truth" or "wheels"


@dataclass(frozen=True)
class JointStates:
    names: tuple[str, ...]
    positions: tuple[float, ...]   # rad
    velocities: tuple[float, ...]  # rad/s


@dataclass(frozen=True)
class Contact:
    robot_part: str
    other: str  # obstacle id, wall name, ...


@dataclass(frozen=True)
class Contacts:
    contacts: tuple[Contact, ...]


@dataclass(frozen=True)
class GroundTruth:
    pose: Pose2D       # world frame
    velocity: Twist2D  # robot frame


# ---- streaming -----------------------------------------------------------
@dataclass(frozen=True)
class SensorInfo:
    """Static description of one sensor, from ``Simulation.sensors()``."""

    name: str
    kind: str          # lidar | camera | imu | odometry | joint_states | contact | ground_truth
    frame: str
    rate_hz: float
    enabled: bool
    mount: Pose3D      # sensor pose relative to the robot base frame
    parent_frame: str  # the robot base frame
    config: Any        # the full spec from asset.yaml (with runtime changes)


@dataclass(frozen=True)
class SensorReading:
    """One sample from one sensor, stamped with simulation time."""

    name: str
    kind: str
    frame: str
    stamp: float
    data: Any  # LaserScan | Image | Imu | Odometry | JointStates | Contacts | GroundTruth


@dataclass(frozen=True)
class Observation:
    """Snapshot after a step: ground truth plus the latest reading of each sensor."""

    time: float
    pose: Pose2D           # ground truth, world frame
    velocity: Twist2D      # ground truth, robot frame
    scan: LaserScan | None = None
    contacts: tuple[Contact, ...] = ()
    images: dict[str, Any] = field(default_factory=dict)  # camera name -> numpy array
    imu: Imu | None = None
    odom: Odometry | None = None
    joints: JointStates | None = None
    readings: dict[str, SensorReading] = field(default_factory=dict)  # latest per sensor

    @property
    def in_collision(self) -> bool:
        return bool(self.contacts)
