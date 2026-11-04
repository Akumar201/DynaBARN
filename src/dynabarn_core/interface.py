"""The robot's middleware interface, independent of simulator and middleware.

Every backend exposes the same topics, message types and frames, whether it
is MuJoCo published by the dynabarn_ros node or Gazebo through ros_gz_bridge.
Both are generated from this one description, so a navigation stack sees
the same robot in either simulator.

Names are relative (no leading slash), so a ROS namespace applies to all of
them. Message types are given as ROS type names; nothing here imports ROS.
"""
from __future__ import annotations

from dataclasses import dataclass

from .assets import CameraSpec, RobotAsset, SensorSpec

CMD_VEL = "cmd_vel"
CLOCK = "/clock"  # always global
TF = "/tf"
TF_STATIC = "/tf_static"


@dataclass(frozen=True)
class Topic:
    name: str      # relative topic name
    ros_type: str  # e.g. "sensor_msgs/msg/LaserScan"
    sensor: str    # dynabarn sensor name ("" for non-sensor topics)
    frame: str     # header.frame_id of published messages


def sensor_topics(robot: RobotAsset, enabled_only: bool = True) -> list[Topic]:
    """Topics published for the robot's sensors."""
    return [t for s in robot.sensors() if s.enabled or not enabled_only for t in topics_for(s, robot.base_frame)]


def topics_for(s: SensorSpec, base_frame: str) -> list[Topic]:
    """Topics of one sensor (an empty list for sensors without a standard ROS message)."""
    k = s.kind
    if k == "lidar":
        return [Topic("scan" if s.name == "lidar" else s.name, "sensor_msgs/msg/LaserScan", s.name, s.frame)]
    if k == "imu":
        return [Topic(s.name, "sensor_msgs/msg/Imu", s.name, s.frame)]
    if k == "odometry":
        return [Topic(s.name, "nav_msgs/msg/Odometry", s.name, s.frame)]
    if k == "joint_states":
        return [Topic(s.name, "sensor_msgs/msg/JointState", s.name, base_frame)]
    if k == "ground_truth":
        return [Topic(s.name, "nav_msgs/msg/Odometry", s.name, s.frame)]
    if k == "camera":
        assert isinstance(s, CameraSpec)
        return [Topic(f"{s.name}/image_raw", "sensor_msgs/msg/Image", s.name, s.optical_frame),
                Topic(f"{s.name}/camera_info", "sensor_msgs/msg/CameraInfo", s.name, s.optical_frame)]
    if k == "contact":
        return []  # no standard ROS message; read it through the dynabarn API
    return []


def odom_child_frame(robot: RobotAsset) -> str:
    """Child frame of the odometry (and of the odom -> base TF)."""
    return robot.base_frame


def static_frames(robot: RobotAsset, enabled_only: bool = False) -> list[tuple[str, str, tuple[float, ...]]]:
    """(parent, child, [x y z roll pitch yaw]) for every sensor frame, including
    each camera's optical frame (z forward, x right, y down)."""
    import math

    out: list[tuple[str, str, tuple[float, ...]]] = []
    seen: set[str] = set()
    for s in robot.sensors():
        if enabled_only and not s.enabled:
            continue
        if s.kind in ("odometry", "ground_truth", "joint_states", "contact") or not s.frame:
            continue
        if s.frame not in seen and s.frame != robot.base_frame:
            out.append((robot.base_frame, s.frame, s.mount_pose))
            seen.add(s.frame)
        if isinstance(s, CameraSpec) and s.optical_frame not in seen:
            out.append((s.frame, s.optical_frame, (0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2)))
            seen.add(s.optical_frame)
    return out


__all__ = ["CMD_VEL", "CLOCK", "TF", "TF_STATIC", "Topic", "sensor_topics", "topics_for", "static_frames", "odom_child_frame"]
