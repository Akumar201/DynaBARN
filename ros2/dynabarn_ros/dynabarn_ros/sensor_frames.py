"""Publish the robot's static sensor frames on /tf_static (used with Gazebo;
the MuJoCo node publishes them itself). Frames come from asset.yaml mounts.

    ros2 run dynabarn_ros sensor_frames --ros-args -p robot:=jackal
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from dynabarn_core import AssetLibrary, interface

from . import convert


class SensorFrames(Node):
    def __init__(self) -> None:
        super().__init__("dynabarn_sensor_frames")
        robot_id = self.declare_parameter("robot", "jackal").value
        assets = self.declare_parameter("assets", "").value
        robot = AssetLibrary([p for p in assets.split(":") if p]).robot(robot_id)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TFMessage, interface.TF_STATIC, qos)
        frames = interface.static_frames(robot)
        self.pub.publish(TFMessage(transforms=[convert.static_transform(p, c, pose) for p, c, pose in frames]))
        self.get_logger().info(f"{robot_id}: {', '.join(c for _, c, _ in frames)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
