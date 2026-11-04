"""dynabarn message types -> ROS 2 messages. Pure functions, no node."""
from __future__ import annotations

import math

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan

from dynabarn_core.assets import CameraSpec
from dynabarn_core.core.messages import (
    GroundTruth,
    Imu as DbImu,
    JointStates,
    LaserScan as DbScan,
    Odometry,
)


def stamp(t: float) -> Time:
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=min(int(round((t - sec) * 1e9)), 999_999_999))


def yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def rpy_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def laser_scan(scan: DbScan, t: float, frame: str, rate_hz: float) -> LaserScan:
    m = LaserScan()
    m.header.stamp, m.header.frame_id = stamp(t), frame
    m.angle_min = float(scan.angle_min)
    m.angle_max = float(scan.angle_max)
    m.angle_increment = float(scan.angle_increment)
    m.scan_time = 1.0 / rate_hz
    m.range_min, m.range_max = float(scan.range_min), float(scan.range_max)
    m.ranges = [float(r) for r in scan.ranges]
    return m


def imu(data: DbImu, t: float, frame: str) -> Imu:
    m = Imu()
    m.header.stamp, m.header.frame_id = stamp(t), frame
    w, x, y, z = data.orientation
    m.orientation = Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(v) for v in data.angular_velocity)
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (
        float(v) for v in data.linear_acceleration)
    return m


def odometry(data: Odometry | GroundTruth, t: float, frame: str, child: str) -> OdometryMsg:
    m = OdometryMsg()
    m.header.stamp, m.header.frame_id = stamp(t), frame
    m.child_frame_id = child
    m.pose.pose.position.x, m.pose.pose.position.y = float(data.pose.x), float(data.pose.y)
    m.pose.pose.orientation = yaw_quat(data.pose.yaw)
    m.twist.twist.linear.x = float(data.velocity.linear)
    m.twist.twist.angular.z = float(data.velocity.angular)
    return m


def odom_tf(data: Odometry, t: float, frame: str, child: str) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id = stamp(t), frame
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y = float(data.pose.x), float(data.pose.y)
    tf.transform.rotation = yaw_quat(data.pose.yaw)
    return tf


def joint_state(data: JointStates, t: float) -> JointState:
    m = JointState()
    m.header.stamp = stamp(t)
    m.name = list(data.names)
    m.position = [float(v) for v in data.positions]
    m.velocity = [float(v) for v in data.velocities]
    return m


def image(img, t: float, frame: str) -> Image:
    m = Image()
    m.header.stamp, m.header.frame_id = stamp(t), frame
    arr = np.ascontiguousarray(img.data)
    m.height, m.width = int(arr.shape[0]), int(arr.shape[1])
    m.encoding = img.encoding
    m.is_bigendian = 0
    m.step = int(arr.strides[0])
    m.data = arr.tobytes()
    return m


def camera_info(spec: CameraSpec, t: float) -> CameraInfo:
    """Pinhole intrinsics from the camera's horizontal field of view (square pixels)."""
    m = CameraInfo()
    m.header.stamp, m.header.frame_id = stamp(t), spec.optical_frame
    m.width, m.height = spec.width, spec.height
    f = (spec.width / 2.0) / math.tan(math.radians(spec.fov_deg) / 2.0)
    cx, cy = spec.width / 2.0, spec.height / 2.0
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    m.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


def static_transform(parent: str, child: str, pose: tuple[float, ...]) -> TransformStamped:
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = parent, child
    x, y, z, roll, pitch, yaw = pose
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
    tf.transform.rotation = rpy_quat(roll, pitch, yaw)
    return tf
