#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def angle_wrap(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('tb3_end_to_end')

        # Parameters (declare with defaults; can be overridden via ROS params)
        self.declare_parameter('goal_x', 10.0)    # meters in odom frame
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('xy_tolerance', 0.10)   # meters
        self.declare_parameter('yaw_tolerance', 0.20)  # radians
        self.declare_parameter('lin_kp', 0.8)
        self.declare_parameter('ang_kp', 2.0)
        self.declare_parameter('max_linear_speed', 0.25)   # m/s
        self.declare_parameter('max_angular_speed', 1.2)   # rad/s
        self.declare_parameter('obstacle_stop_dist', 0.35) # meters; set <=0 to disable

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.xy_tol = float(self.get_parameter('xy_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        self.lin_kp = float(self.get_parameter('lin_kp').value)
        self.ang_kp = float(self.get_parameter('ang_kp').value)
        self.vmax = float(self.get_parameter('max_linear_speed').value)
        self.wmax = float(self.get_parameter('max_angular_speed').value)
        self.stop_dist = float(self.get_parameter('obstacle_stop_dist').value)

        # State
        self.odom: Optional[Odometry] = None
        self.min_scan: Optional[float] = None
        self.reached = False
        self.stop_sent_time = None

        # I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        # Loop
        self.timer = self.create_timer(1.0/20.0, self._control_step)  # 20 Hz

        self.get_logger().info(
            f"End-to-end: goal=({self.goal_x:.2f}, {self.goal_y:.2f}) in 'odom'"
        )
        if self.stop_dist <= 0.0:
            self.get_logger().info("Obstacle stop disabled.")
        else:
            self.get_logger().info(f"Obstacle stop enabled at {self.stop_dist:.2f} m")

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_scan(self, msg: LaserScan):
        # Track the minimum forward range to do a simple safety stop
        if len(msg.ranges) == 0:
            self.min_scan = None
            return
        # Focus a forward sector (e.g., ±20°) for a bit more realism
        total = len(msg.ranges)
        angle_span = math.radians(20.0)
        center_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        half_span_idx = int(angle_span / msg.angle_increment)
        i0 = clamp(center_idx - half_span_idx, 0, total - 1)
        i1 = clamp(center_idx + half_span_idx, 0, total - 1)
        sector = [r for r in msg.ranges[i0:i1+1] if not math.isinf(r) and not math.isnan(r)]
        self.min_scan = min(sector) if sector else None

    def _control_step(self):
        if self.reached:
            # After reaching, send a few zero commands then shut down node
            self._publish_stop()
            if self.stop_sent_time is None:
                self.stop_sent_time = self.get_clock().now()
            elif (self.get_clock().now() - self.stop_sent_time) > Duration(seconds=1.0):
                self.get_logger().info("Goal reached. Stopping.")
                rclpy.shutdown()
            return

        if self.odom is None:
            return  # wait for odom

        # Current pose from odom
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Position error
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = angle_wrap(desired_yaw - yaw)

        # Check goal condition
        if dist < self.xy_tol and abs(yaw_err) < self.yaw_tol:
            self.reached = True
            self._publish_stop()
            return

        # Simple obstacle stop
        obstacle_blocking = False
        if self.stop_dist > 0.0 and self.min_scan is not None:
            obstacle_blocking = self.min_scan < self.stop_dist

        # P-control
        cmd = Twist()

        # Rotate toward the goal more aggressively when far off-heading
        w = clamp(self.ang_kp * yaw_err, -self.wmax, self.wmax)

        v_cmd = self.lin_kp * dist
        # Slow down if heading error is large
        v_cmd *= max(0.0, math.cos(yaw_err))  # negative cos -> zero forward
        v = clamp(v_cmd, 0.0, self.vmax)

        if obstacle_blocking:
            v = 0.0

        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

def main():
    rclpy.init()
    node = GoToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
