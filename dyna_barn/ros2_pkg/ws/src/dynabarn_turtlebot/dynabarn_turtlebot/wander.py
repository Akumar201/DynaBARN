#!/usr/bin/env python3
import math
import random
import time
from collections import namedtuple, deque

import numpy as np
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

Node = namedtuple("Node", "x y parent")


def yaw_from_quaternion(x, y, z, w):
    """Return yaw (rad) from geometry_msgs/Quaternion components."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalGrid:
    def __init__(self, size_m=20.0, res=0.05, inflate=0.15):
        self.size_m = size_m
        self.res = res
        self.n = int(size_m / res)
        self.center = self.n // 2
        self.inflate_cells = max(1, int(inflate / res))
        self.grid = np.zeros((self.n, self.n), dtype=np.uint8)

    def reset(self):
        self.grid.fill(0)

    def world_to_grid(self, x, y):
        gx = int(round(x / self.res)) + self.center
        gy = int(round(y / self.res)) + self.center
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.n and 0 <= gy < self.n

    def mark_occ(self, gx, gy):
        if self.in_bounds(gx, gy):
            self.grid[gx, gy] = 1

    def inflate(self):
        k = self.inflate_cells
        if k <= 0:
            return
        # Square dilation is separable: dilate along x, then along y
        # (2*(2k+1) slice ops instead of (2k+1)^2)
        src = self.grid
        tmp = src.copy()
        for d in range(-k, k + 1):
            s, e = max(0, -d), min(self.n, self.n - d)
            if s < e:
                np.maximum(tmp[s:e, :], src[s + d:e + d, :], out=tmp[s:e, :])
        dst = tmp.copy()
        for d in range(-k, k + 1):
            s, e = max(0, -d), min(self.n, self.n - d)
            if s < e:
                np.maximum(dst[:, s:e], tmp[:, s + d:e + d], out=dst[:, s:e])
        self.grid = dst

    def collision(self, gx, gy):
        """True if occupied or OOB."""
        return (not self.in_bounds(gx, gy)) or (self.grid[gx, gy] != 0)


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class RRTPlanner:
    def __init__(self, grid, step_cells=3, iters=5000):
        self.g = grid
        self.step = step_cells
        self.iters = iters

    def _steer(self, a, b):
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
        dx, dy = bx - ax, by - ay
        d = max(1, int(round((dx * dx + dy * dy) ** 0.5)))
        sx, sy = dx / float(d), dy / float(d)
        nx = int(round(ax + sx * self.step))
        ny = int(round(ay + sy * self.step))
        return Node(nx, ny, None)

    def _free_segment(self, p, q):
        for cx, cy in bresenham(p.x, p.y, q.x, q.y):
            if self.g.collision(cx, cy):
                return False
        return True

    def plan(self, start_g, goal_g, goal_tol=3):
        if self.g.collision(*start_g) or self.g.collision(*goal_g):
            return None
        nodes = [Node(start_g[0], start_g[1], -1)]
        # Node coordinates mirrored in arrays for vectorized nearest-neighbor search
        xs = np.empty(self.iters + 1, dtype=np.int64)
        ys = np.empty(self.iters + 1, dtype=np.int64)
        xs[0], ys[0] = start_g
        for _ in range(self.iters):
            if random.random() < 0.10:
                sx, sy = goal_g
            else:
                sx = random.randint(0, self.g.n - 1)
                sy = random.randint(0, self.g.n - 1)
            cnt = len(nodes)
            i_near = int(np.argmin((xs[:cnt] - sx) ** 2 + (ys[:cnt] - sy) ** 2))
            nnear = nodes[i_near]
            nnew = self._steer(nnear, Node(sx, sy, None))
            if self.g.collision(nnew.x, nnew.y):
                continue
            if not self._free_segment(nnear, nnew):
                continue
            xs[cnt], ys[cnt] = nnew.x, nnew.y
            nodes.append(Node(nnew.x, nnew.y, i_near))
            if (nnew.x - goal_g[0]) ** 2 + (nnew.y - goal_g[1]) ** 2 <= goal_tol ** 2:
                nodes.append(Node(goal_g[0], goal_g[1], len(nodes) - 1))
                path = []
                idx = len(nodes) - 1
                while idx != -1:
                    n = nodes[idx]
                    path.append((n.x, n.y))
                    idx = n.parent
                path.reverse()
                return path
        return None


class WanderRRT(RclNode):
    def __init__(self):
        super().__init__("rrt_wander")

        # Parameters (declare so they can be overridden by launch)
        self.declare_parameter("grid_size", 20.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("inflate", 0.18)
        self.declare_parameter("rrt_step_m", 0.15)
        self.declare_parameter("rrt_iters", 5000)
        self.declare_parameter("goal_radius_m", 0.4)
        self.declare_parameter("sample_radius_m", 5.0)
        self.declare_parameter("min_goal_clear_m", 0.4)
        self.declare_parameter("lin_vel", 0.25)
        self.declare_parameter("ang_vel", 1.2)
        self.declare_parameter("look_ahead_m", 0.6)
        self.declare_parameter("replan_sec", 3.0)

        gs = float(self.get_parameter("grid_size").value)
        res = float(self.get_parameter("resolution").value)
        inflate = float(self.get_parameter("inflate").value)
        self.grid = LocalGrid(size_m=gs, res=res, inflate=inflate)

        self.step_cells = max(2, int(float(self.get_parameter("rrt_step_m").value) / self.grid.res))
        self.max_iters = int(self.get_parameter("rrt_iters").value)
        self.goal_radius = float(self.get_parameter("goal_radius_m").value)
        self.sample_radius = float(self.get_parameter("sample_radius_m").value)
        self.min_goal_clear = float(self.get_parameter("min_goal_clear_m").value)

        self.v = float(self.get_parameter("lin_vel").value)
        self.w_max = float(self.get_parameter("ang_vel").value)
        self.look_ahead = float(self.get_parameter("look_ahead_m").value)
        self.replan_sec = float(self.get_parameter("replan_sec").value)

        # State
        self.pose = None
        self.yaw = 0.0
        self.scan = None
        self.last_plan = None
        self.plan_idx = 0
        self.last_plan_time = 0.0

        # QoS: Lidar often uses best-effort; odom usually reliable
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_default = 10

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, qos_default)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_scan)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos_default)

        # 15 Hz control/update loop
        self.timer = self.create_timer(1.0 / 15.0, self.step)

        self.get_logger().info("WanderRRT (ROS 2) started.")

    # ---- Callbacks ----
    def odom_cb(self, msg: Odometry):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ---- Core logic ----
    def build_grid_from_scan(self):
        self.grid.reset()
        if self.scan is None:
            return
        ranges = np.asarray(self.scan.ranges, dtype=np.float64)
        angles = self.scan.angle_min + self.scan.angle_increment * np.arange(ranges.size)
        valid = np.isfinite(ranges) & (ranges <= self.scan.range_max)
        r, a = ranges[valid], angles[valid]
        g = self.grid
        gx = np.rint(r * np.cos(a) / g.res).astype(np.int64) + g.center
        gy = np.rint(r * np.sin(a) / g.res).astype(np.int64) + g.center
        inb = (gx >= 0) & (gx < g.n) & (gy >= 0) & (gy < g.n)
        g.grid[gx[inb], gy[inb]] = 1
        g.inflate()

    def pick_random_goal(self):
        for _ in range(200):
            R = random.uniform(1.0, self.sample_radius)
            th = random.uniform(-math.pi, math.pi)
            gx, gy = self.grid.world_to_grid(R * math.cos(th), R * math.sin(th))
            if self.grid.in_bounds(gx, gy) and not self.grid.collision(gx, gy):
                clr = max(1, int(self.min_goal_clear / self.grid.res))
                sub = self.grid.grid[max(0, gx - clr) : gx + clr + 1, max(0, gy - clr) : gy + clr + 1]
                if sub.size and sub.max() == 0:
                    return (gx, gy)
        return None

    def pure_pursuit(self, target_xy):
        tx, ty = target_xy
        dx = (tx - self.grid.center) * self.grid.res
        dy = (ty - self.grid.center) * self.grid.res
        if math.hypot(dx, dy) < 0.1:
            return 0.0, 0.0
        ang = math.atan2(dy, dx)        # robot-centric grid
        ang_err = math.atan2(math.sin(ang), math.cos(ang))
        w = max(-self.w_max, min(self.w_max, 2.0 * ang_err))
        v = self.v * (0.5 + 0.5 * math.exp(-abs(ang_err)))
        return v, w

    def step(self):
        if self.pose is None or self.scan is None:
            return

        self.build_grid_from_scan()

        need_replan = (
            self.last_plan is None
            or (time.time() - self.last_plan_time > self.replan_sec)
            or (self.plan_idx >= len(self.last_plan))
        )

        if need_replan:
            start_g = (self.grid.center, self.grid.center)
            goal_g = self.pick_random_goal()
            if goal_g is None:
                self.pub_cmd.publish(Twist())
                return
            planner = RRTPlanner(self.grid, step_cells=self.step_cells, iters=self.max_iters)
            path = planner.plan(start_g, goal_g, goal_tol=max(2, int(self.goal_radius / self.grid.res)))
            if path is None or len(path) < 3:
                return
            self.last_plan = path
            self.plan_idx = min(2, len(path) - 1)  # skip first couple of points
            self.last_plan_time = time.time()

        wp = self.last_plan[self.plan_idx]
        v, w = self.pure_pursuit(wp)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        rx, ry = self.grid.world_to_grid(0.0, 0.0)
        if (wp[0] - rx) ** 2 + (wp[1] - ry) ** 2 < (max(2, int(self.look_ahead / self.grid.res))) ** 2:
            self.plan_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = WanderRRT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
