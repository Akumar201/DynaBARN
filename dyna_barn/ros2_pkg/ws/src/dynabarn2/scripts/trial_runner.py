#!/usr/bin/env python3
"""Runs one navigation trial and appends the result to a CSV file.

Follows The BARN Challenge rules: the clock starts when the robot first moves; a trial ends on
reaching 1 m from the goal (succeeded), touching anything (collided) or the time limit (timeout).

Score = success * optimal_time / clip(time, 2 * optimal_time, 8 * optimal_time), where
optimal_time = path length / 2 m/s. The path length is the start-goal distance: the ROS 1 runner
computes this as a negative number for dynamic worlds, so its DynaBARN scores are not comparable.
"""
import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

ROBOT_MODEL = 'robot'
ROBOT_SPEED = 2.0   # m/s, the challenge's Jackal limit
GOAL_RADIUS = 1.0   # m
START_RADIUS = 0.1  # m the robot must move before the clock starts


def nav_metric(success, elapsed, path_length):
    optimal = path_length / ROBOT_SPEED
    return int(success) * optimal / np.clip(elapsed, 2 * optimal, 8 * optimal)


class TrialRunner(Node):

    def __init__(self):
        super().__init__('trial_runner', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        for name, default in [('world', ''), ('planner', 'none'), ('out_file', 'results.csv'),
                              ('timeout', 100.0), ('start_timeout', 60.0), ('send_goal', True),
                              ('x', 11.0), ('y', 0.0), ('yaw', math.pi),
                              ('goal_x', -9.0), ('goal_y', 0.0)]:
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.start = (p('x'), p('y'))
        self.goal = (p('goal_x'), p('goal_y'))
        self.yaw = p('yaw')
        self.timeout = p('timeout')
        self.start_timeout = p('start_timeout')
        self.path_length = math.dist(self.start, self.goal)

        self.pos = None
        self.touched = False
        self.t_begin = None    # sim time at which we started waiting for the robot to move
        self.t_start = None    # sim time at which the robot started moving
        self.finished = False
        self.nav = None
        if p('send_goal') and p('planner') != 'none':
            self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.t_last_try = -1.0

        self.create_subscription(TFMessage, '/model/robot/pose', self.on_pose, 10)
        self.create_subscription(Bool, '/robot/touched', self.on_touch, 10)
        self.create_timer(0.02, self.step)

    def on_pose(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == ROBOT_MODEL:
                t = tf.transform.translation
                self.pos = (t.x, t.y)

    def on_touch(self, msg):
        if msg.data and self.t_start is not None:
            self.touched = True

    def send_goal(self):
        # Nav2 runs in the odom frame, which starts at the robot's spawn pose.
        dx, dy = self.goal[0] - self.start[0], self.goal[1] - self.start[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'odom'
        goal.pose.pose.position.x = c * dx + s * dy
        goal.pose.pose.position.y = -s * dx + c * dy
        goal.pose.pose.orientation.w = 1.0
        self.goal_sent = True
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        # Nav2's action server answers before it is active, and rejects goals until then.
        if future.result().accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.goal_sent = False

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.finished or now == 0.0 or self.pos is None:
            return
        if self.t_begin is None:
            self.t_begin = now
            self.get_logger().info(f'Robot at {self.pos}; waiting for it to move')
        if self.nav is not None and not self.goal_sent and self.nav.server_is_ready() \
                and now - self.t_last_try >= 1.0:
            self.t_last_try = now
            self.send_goal()

        if self.t_start is None:
            if math.dist(self.pos, self.start) >= START_RADIUS:
                self.t_start = now
                self.get_logger().info('Trial running')
            elif now - self.t_begin > self.start_timeout:
                self.finish('no_start', 0.0)
            return

        elapsed = now - self.t_start
        if self.touched:
            self.finish('collided', elapsed)
        elif math.dist(self.pos, self.goal) <= GOAL_RADIUS:
            self.finish('succeeded', elapsed)
        elif elapsed >= self.timeout:
            self.finish('timeout', elapsed)

    def finish(self, status, elapsed):
        self.finished = True
        metric = nav_metric(status == 'succeeded', elapsed, self.path_length)
        self.get_logger().info(f'Trial {status} in {elapsed:.3f} s, score {metric:.4f}')
        path = self.get_parameter('out_file').value
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['world', 'planner', 'status', 'time_s', 'score',
                            'final_x', 'final_y'])
            w.writerow([self.get_parameter('world').value, self.get_parameter('planner').value,
                        status, f'{elapsed:.3f}', f'{metric:.4f}',
                        f'{self.pos[0]:.3f}', f'{self.pos[1]:.3f}'])
        raise SystemExit


def main():
    rclpy.init()
    node = TrialRunner()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
