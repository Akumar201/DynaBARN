#!/usr/bin/env python3
"""Drives the robot with a learned policy (planner:=learned), and in training mode records the
episode for the learner.

The policy sees only what a real robot has: the lidar and its own odometry. The true pose and
contact sensor are recorded only to compute rewards offline.

Parameters (also read from the environment by run_trial.launch.py):
  policy_file   exported actor (.npz). Empty or missing -> a noisy go-to-goal heuristic,
                used to seed training before the first policy exists.
  mode          eval: deterministic actions. train: sampled actions, episode saved to episode_dir.
"""
import math
import os
import socket
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from dynabarn2_learned import core

SCAN_TOPIC = '/sensors/lidar2d_0/scan'
ODOM_TOPIC = '/platform/odom/filtered'
CMD_TOPIC = '/platform/cmd_vel'
GOAL_RADIUS = 1.0


class LearnedPolicy(Node):

    def __init__(self):
        super().__init__('learned_policy', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        for name, default in [('policy_file', ''), ('mode', 'eval'), ('episode_dir', ''),
                              ('world', ''), ('reactive', 'off'), ('seed', -1),
                              ('x', 11.0), ('y', 0.0), ('yaw', math.pi),
                              ('goal_x', -9.0), ('goal_y', 0.0)]:
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.mode = p('mode')
        self.goal_world = (p('goal_x'), p('goal_y'))
        # Goal in the odom frame, which starts at the spawn pose (as in trial_runner.py).
        dx, dy = p('goal_x') - p('x'), p('goal_y') - p('y')
        c, s = math.cos(p('yaw')), math.sin(p('yaw'))
        self.goal_odom = (c * dx + s * dy, -s * dx + c * dy)
        seed = p('seed') if p('seed') >= 0 else int.from_bytes(os.urandom(4), 'little')
        self.rng = np.random.default_rng(seed)

        self.actor = None
        path = p('policy_file')
        if path and os.path.exists(path):
            self.actor = core.NumpyActor(path)
            self.get_logger().info(f'Policy {path} (version {self.actor.version}), mode {self.mode}')
        elif self.mode == 'eval':
            raise RuntimeError(f'mode eval needs a policy file, got "{path}"')
        else:
            self.get_logger().info('No policy yet: using the noisy heuristic')

        self.frames = []
        self.min_scan = core.SCAN_RANGE
        self.odom = None
        self.true_pos = None
        self.start_pos = None
        self.touched = False
        self.last_cmd = (0.0, 0.0)
        self.finished = False
        self.saved = False
        self.log = {'obs': [], 'act': [], 'pos': [], 'min_scan': []}

        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(TFMessage, '/model/robot/pose', self.on_pose, 10)
        self.create_subscription(Bool, '/robot/touched', self.on_touch, 10)
        self.create_timer(1.0 / core.CONTROL_HZ, self.step)

    def on_scan(self, msg):
        self.latest_scan = core.bin_scan(msg.ranges, msg.angle_min, msg.angle_increment)
        self.min_scan = float(self.latest_scan.min() * core.SCAN_RANGE)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                     msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_pose(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == 'robot':
                self.true_pos = (tf.transform.translation.x, tf.transform.translation.y)
                if self.start_pos is None:
                    self.start_pos = self.true_pos

    def on_touch(self, msg):
        if msg.data:
            self.touched = True

    def publish(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.pub.publish(cmd)

    def observe(self):
        self.frames.insert(0, self.latest_scan)
        del self.frames[core.FRAMES:]
        x, y, yaw, v, w = self.odom
        goal = core.goal_features(x, y, yaw, *self.goal_odom)
        return core.build_obs(self.frames, goal, (v, w), self.last_cmd), goal

    def step(self):
        if self.finished:
            self.publish(0.0, 0.0)
            return
        if not hasattr(self, 'latest_scan') or self.odom is None or self.true_pos is None:
            return
        obs, goal = self.observe()
        self.log['obs'].append(obs)
        self.log['pos'].append(self.true_pos)
        self.log['min_scan'].append(self.min_scan)
        if self.touched or math.dist(self.true_pos, self.goal_world) <= GOAL_RADIUS:
            self.finished = True
            self.publish(0.0, 0.0)
            self.save()
            return
        if self.actor is not None:
            action = self.actor.act(obs, None if self.mode == 'eval' else self.rng)
        else:
            front = float(self.latest_scan[core.SCAN_BINS // 3: 2 * core.SCAN_BINS // 3].min()
                          * core.SCAN_RANGE)
            action = core.heuristic_action(goal, front, self.rng)
        self.log['act'].append(np.asarray(action, np.float32))
        v, w = core.to_command(action)
        self.last_cmd = (v, w)
        self.publish(v, w)

    def status(self):
        moved = self.start_pos is not None and self.true_pos is not None and \
            math.dist(self.start_pos, self.true_pos) >= 0.1
        if self.touched:
            return 'collided' if moved else 'hit_at_start'
        if self.true_pos is not None and math.dist(self.true_pos, self.goal_world) <= GOAL_RADIUS:
            return 'succeeded'
        return 'cut'   # stopped from outside (time limit or shutdown): not a terminal state

    def save(self):
        if self.saved or self.mode != 'train' or not self.get_parameter('episode_dir').value:
            return
        self.saved = True
        n = len(self.log['act'])
        if n < 5:
            return
        out_dir = self.get_parameter('episode_dir').value
        os.makedirs(out_dir, exist_ok=True)
        name = f'{socket.gethostname()}_{os.getpid()}_{time.time_ns()}'
        tmp = os.path.join(out_dir, f'.{name}.tmp.npz')
        np.savez_compressed(
            tmp, obs=np.asarray(self.log['obs'][:n + 1], np.float32),
            act=np.asarray(self.log['act'], np.float32),
            pos=np.asarray(self.log['pos'][:n + 1], np.float32),
            min_scan=np.asarray(self.log['min_scan'][:n + 1], np.float32),
            status=self.status(), world=self.get_parameter('world').value,
            reactive=self.get_parameter('reactive').value,
            policy_version=self.actor.version if self.actor else -1)
        os.replace(tmp, os.path.join(out_dir, f'{name}.npz'))
        self.get_logger().info(f'Saved episode: {n} steps, {self.status()}')


def main():
    rclpy.init()
    node = LearnedPolicy()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    if len(node.log['obs']) > len(node.log['act']) or not node.saved:
        # Ended from outside: make sure obs has one more entry than act before saving.
        if len(node.log['obs']) == len(node.log['act']) and node.log['act']:
            node.log['act'].pop()
        node.save()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
