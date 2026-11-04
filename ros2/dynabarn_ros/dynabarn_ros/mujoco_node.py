"""Run a dynabarn scenario in MuJoCo and expose it on ROS 2.

    ros2 run dynabarn_ros mujoco_node --ros-args -p preset:=dynabarn_lanes -p seed:=3

Publishes (relative names, so a namespace applies):
    /clock                    simulation time (use_sim_time:=true elsewhere)
    scan, imu, odom, joint_states, ground_truth, <camera>/image_raw + camera_info
    /tf (odom -> base_link), /tf_static (sensor frames)
Subscribes:
    cmd_vel (geometry_msgs/Twist)

Topics, types and frames come from ``dynabarn_core.interface``, the same
description the Gazebo bridge is generated from. Every message is stamped
with simulation time and published from the simulation thread the moment
the reading exists, so rates follow the sensor settings in asset.yaml.
"""
from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from dynabarn_core import DynaBarn, interface
from dynabarn_core.core.messages import SensorReading

from . import convert
from .common import make_scenario

_TYPES = {
    "sensor_msgs/msg/LaserScan": ("sensor_msgs.msg", "LaserScan"),
    "sensor_msgs/msg/Imu": ("sensor_msgs.msg", "Imu"),
    "nav_msgs/msg/Odometry": ("nav_msgs.msg", "Odometry"),
    "sensor_msgs/msg/JointState": ("sensor_msgs.msg", "JointState"),
    "sensor_msgs/msg/Image": ("sensor_msgs.msg", "Image"),
    "sensor_msgs/msg/CameraInfo": ("sensor_msgs.msg", "CameraInfo"),
}


def _msg_class(ros_type: str):
    import importlib

    mod, name = _TYPES[ros_type]
    return getattr(importlib.import_module(mod), name)


class MujocoNode(Node):
    def __init__(self) -> None:
        super().__init__("dynabarn_mujoco")
        p = {
            "scenario": "", "preset": "", "generator": "", "params": "", "seed": 0, "assets": "",
            "enable": "", "disable": "", "realtime": 1.0, "command_timeout": 0.5,
            "viewer": False, "follow": False, "publish_tf": True,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = {k: self.get_parameter(k).value for k in p}

        scenario, lib = make_scenario(g["scenario"], g["preset"], g["generator"], g["params"],
                                      int(g["seed"]), g["assets"])
        enable = [s for s in g["enable"].split(",") if s]
        disable = [s for s in g["disable"].split(",") if s]
        self.db = DynaBarn(scenario, assets=lib, realtime=(g["realtime"] or None),
                           command_timeout=(g["command_timeout"] or None),
                           enable=enable, disable=disable)
        self.robot = lib.robot(scenario.robot.asset)
        self.base = self.robot.base_frame
        self.publish_tf = bool(g["publish_tf"])

        # one publisher per topic of every enabled sensor
        self.pubs: dict[str, list] = {}
        self.specs = {}
        for name, info in self.db.sensors().items():
            if not info.enabled:
                continue
            self.specs[name] = info.config
            for t in interface.topics_for(info.config, self.base):
                self.pubs.setdefault(name, []).append((t, self.create_publisher(_msg_class(t.ros_type), t.name, 10)))
        self.clock_pub = self.create_publisher(Clock, interface.CLOCK, 10)
        self.tf_pub = self.create_publisher(TFMessage, interface.TF, 50)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.create_publisher(TFMessage, interface.TF_STATIC, static_qos)
        self.tf_static_pub.publish(TFMessage(transforms=[
            convert.static_transform(parent, child, pose) for parent, child, pose in interface.static_frames(self.robot)
        ]))
        self.create_subscription(Twist, interface.CMD_VEL, self._on_cmd, 10)

        self._clock_lock = threading.Lock()
        self._last_clock = -1.0
        self.db.subscribe("*", self._on_reading)
        self.create_timer(0.01, self._clock_tick)  # keeps /clock going even without fast sensors
        if g["viewer"]:
            self.db.open_viewer(follow=bool(g["follow"]))
        self.db.start()
        topics = sorted(t.name for lst in self.pubs.values() for t, _ in lst)
        self.get_logger().info(f"scenario '{scenario.name}' ({len(scenario.obstacles)} obstacles), robot "
                               f"'{self.robot.id}'; publishing {', '.join(topics)}")

    # -- simulation -> ROS (runs on the simulation thread) ----------------------------
    def _publish_clock(self, t: float) -> None:
        with self._clock_lock:
            if t <= self._last_clock:
                return
            self._last_clock = t
        self.clock_pub.publish(Clock(clock=convert.stamp(t)))

    def _clock_tick(self) -> None:
        self._publish_clock(self.db.time)

    def _on_reading(self, r: SensorReading) -> None:
        self._publish_clock(r.stamp)
        for topic, pub in self.pubs.get(r.name, ()):
            msg = self._convert(topic, r)
            if msg is not None:
                pub.publish(msg)
        if r.kind == "odometry" and self.publish_tf:
            self.tf_pub.publish(TFMessage(transforms=[
                convert.odom_tf(r.data, r.stamp, r.frame, interface.odom_child_frame(self.robot))]))

    def _convert(self, topic: interface.Topic, r: SensorReading):
        spec, t, k = self.specs[r.name], r.stamp, topic.ros_type
        if k.endswith("LaserScan"):
            return convert.laser_scan(r.data, t, r.frame, spec.rate_hz)
        if k.endswith("Imu"):
            return convert.imu(r.data, t, r.frame)
        if k.endswith("Odometry"):
            return convert.odometry(r.data, t, r.frame, self.base)
        if k.endswith("JointState"):
            return convert.joint_state(r.data, t)
        if k.endswith("CameraInfo"):
            return convert.camera_info(spec, t)
        if k.endswith("Image"):
            return convert.image(r.data, t, spec.optical_frame)
        return None

    # -- ROS -> simulation ---------------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        self.db.set_command(msg.linear.x, msg.angular.z)

    def destroy_node(self) -> None:
        self.db.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
