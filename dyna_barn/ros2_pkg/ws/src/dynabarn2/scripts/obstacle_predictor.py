#!/usr/bin/env python3
"""Track moving obstacles from the lidar and publish where they are about to be.

Scan points are clustered into objects, tracked in the odom frame with an alpha-beta filter, and each
moving track is extrapolated at constant velocity. The swept positions go out as a point cloud that
the local costmap marks as obstacles, so an unmodified controller plans around them, and the tracks
go out on predicted_tracks for PredictionCritic, which compares them with trajectories step by step.

It uses the lidar only, with no simulator truth, so it can run on a real robot.
"""
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from tf2_ros import Buffer, TransformException, TransformListener

MAX_RANGE = 6.0        # m, ignore returns further than this
CLUSTER_GAP = 0.3      # m, neighbouring beams further apart than this are separate objects
MAX_WIDTH = 1.6        # m, anything wider is treated as a wall
MIN_POINTS = 2
GATE = 0.8             # m, furthest a track can associate with a detection
ALPHA, BETA = 0.5, 0.25
MIN_HITS = 4           # detections before a track is trusted
MAX_MISSES = 5         # scans a track survives unseen
MIN_SPEED = 0.2        # m/s, slower tracks are not extrapolated
MAX_SPEED = 3.0        # m/s, faster estimates are association errors
HORIZON = 1.5          # s, how far ahead to predict
STEP = 0.2             # s, between predicted positions
BLOB_RADIUS = 0.2      # m, size of each marked position
FRAME = 'odom'
RING = [(BLOB_RADIUS * math.cos(a * math.pi / 3), BLOB_RADIUS * math.sin(a * math.pi / 3))
        for a in range(6)] + [(0.0, 0.0)]


class Track:
    def __init__(self, x, y, t):
        self.x, self.y, self.vx, self.vy, self.t = x, y, 0.0, 0.0, t
        self.hits, self.misses = 1, 0

    def predict(self, t):
        dt = t - self.t
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, x, y, t):
        dt = t - self.t
        if dt <= 0.0:
            return
        px, py = self.predict(t)
        rx, ry = x - px, y - py
        self.x, self.y = px + ALPHA * rx, py + ALPHA * ry
        self.vx += BETA * rx / dt
        self.vy += BETA * ry / dt
        self.t = t
        self.hits += 1
        self.misses = 0

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


def clusters(points):
    """Split ordered scan points into centroids of objects narrow enough to be obstacles."""
    groups, current = [], []
    for p in points:
        if current and math.dist(p, current[-1]) > CLUSTER_GAP:
            groups.append(current)
            current = []
        current.append(p)
    if current:
        groups.append(current)
    found = []
    for g in groups:
        if len(g) >= MIN_POINTS and math.dist(g[0], g[-1]) <= MAX_WIDTH:
            found.append((sum(p[0] for p in g) / len(g), sum(p[1] for p in g) / len(g)))
    return found


class Predictor(Node):
    def __init__(self):
        super().__init__('obstacle_predictor',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('scan_topic', '/sensors/lidar2d_0/scan')
        self.tracks = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(PointCloud2, 'predicted_obstacles', 10)
        self.track_output = self.create_publisher(Float64MultiArray, 'predicted_tracks', 10)
        # Scans can arrive before their transform, so they wait here until it does.
        self.pending = deque(maxlen=20)
        log_path = os.environ.get('DYNABARN_TRACK_LOG')  # evaluation only: where to record tracks
        self.log = open(log_path, 'w') if log_path else None
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 lambda scan: self.pending.append(scan), qos_profile_sensor_data)
        self.create_timer(0.01, self.drain)

    def drain(self):
        while self.pending:
            scan = self.pending[0]
            try:
                tf = self.buffer.lookup_transform(FRAME, scan.header.frame_id,
                                                  Time.from_msg(scan.header.stamp))
            except TransformException:
                return
            self.pending.popleft()
            self.on_scan(scan, tf)

    def on_scan(self, scan, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        points = []
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < min(scan.range_max, MAX_RANGE):
                a = scan.angle_min + i * scan.angle_increment
                lx, ly = r * math.cos(a), r * math.sin(a)
                points.append((t.x + c * lx - s * ly, t.y + s * lx + c * ly))
        now = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        self.associate(clusters(points), now)
        self.publish(scan.header.stamp, now)

    def associate(self, detections, now):
        pairs = sorted((math.dist(track.predict(now), d), i, j)
                       for i, track in enumerate(self.tracks) for j, d in enumerate(detections))
        used_tracks, used_detections = set(), set()
        for distance, i, j in pairs:
            if distance <= GATE and i not in used_tracks and j not in used_detections:
                self.tracks[i].update(*detections[j], now)
                used_tracks.add(i)
                used_detections.add(j)
        for i, track in enumerate(self.tracks):
            if i not in used_tracks:
                track.misses += 1
        self.tracks = [k for k in self.tracks if k.misses <= MAX_MISSES and k.speed <= MAX_SPEED]
        self.tracks += [Track(*d, now) for j, d in enumerate(detections) if j not in used_detections]

    def publish(self, stamp, now):
        points = []
        moving = [k for k in self.tracks if k.hits >= MIN_HITS and k.speed >= MIN_SPEED]
        for track in moving:
            for n in range(1, int(HORIZON / STEP) + 1):
                x, y = track.predict(now + n * STEP)
                points += [(x + dx, y + dy, 0.3) for dx, dy in RING]
        if self.log:
            for track in moving:
                self.log.write(f'{now:.3f} {track.x:.3f} {track.y:.3f} {track.vx:.3f} {track.vy:.3f}\n')
            self.log.flush()
        data = [now]
        for track in moving:
            data += [track.x, track.y, track.vx, track.vy]
        self.track_output.publish(Float64MultiArray(data=data))
        self.output.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=FRAME), points))


def main():
    rclpy.init()
    rclpy.spin(Predictor())


if __name__ == '__main__':
    main()
