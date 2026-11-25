"""Observation, action, reward and a numpy actor for the learned planner.

Observation (OBS_DIM floats, all roughly in [-1, 1]):
  FRAMES lidar frames, newest first, each min-pooled into SCAN_BINS angular bins over the
  Hokuyo's 270 degrees and divided by SCAN_RANGE; then goal distance / 20, cos and sin of the
  goal bearing in the robot frame; measured forward speed and yaw rate; last command.
Stacking frames lets a memoryless network see how obstacles move.

Action: two numbers in [-1, 1] -> forward speed in [V_MIN, V_MAX] m/s and yaw rate in
[-W_MAX, W_MAX] rad/s, the same limits the Nav2 baselines use.
"""
import math

import numpy as np

CONTROL_HZ = 10.0
SCAN_BINS = 72
SCAN_RANGE = 5.0            # m
SCAN_FOV = (-2.356194490192345, 2.356194490192345)
FRAMES = 4
FRAME_STRIDE = 1           # control steps between stacked frames (a policy file may override)
V_MIN, V_MAX = -0.3, 1.0    # m/s
W_MAX = 1.9                 # rad/s
GOAL_SCALE = 20.0
OBS_DIM = SCAN_BINS * FRAMES + 3 + 2 + 2
ACT_DIM = 2

# Reward, per 0.1 s step
PROGRESS = 1.0              # per metre closer to the goal
STEP_COST = 0.01
CLOSE_COST = 0.1            # scaled by how far inside CLOSE_DIST the nearest lidar return is
CLOSE_DIST = 0.6            # m
SUCCESS_REWARD = 10.0
COLLISION_COST = 10.0


def bin_scan(ranges, angle_min, angle_increment):
    """Min-pool one LaserScan into SCAN_BINS bins, normalised to [0, 1]."""
    r = np.asarray(ranges, dtype=np.float32)
    r = np.where(np.isfinite(r) & (r > 0.05), r, SCAN_RANGE)
    r = np.minimum(r, SCAN_RANGE)
    ang = angle_min + angle_increment * np.arange(len(r))
    idx = ((ang - SCAN_FOV[0]) / (SCAN_FOV[1] - SCAN_FOV[0]) * SCAN_BINS).astype(int)
    keep = (idx >= 0) & (idx < SCAN_BINS)
    out = np.full(SCAN_BINS, SCAN_RANGE, dtype=np.float32)
    np.minimum.at(out, idx[keep], r[keep])
    return out / SCAN_RANGE


def goal_features(x, y, yaw, gx, gy):
    dx, dy = gx - x, gy - y
    bearing = math.atan2(dy, dx) - yaw
    return [min(math.hypot(dx, dy), GOAL_SCALE) / GOAL_SCALE, math.cos(bearing), math.sin(bearing)]


def build_obs(frames, goal, vel, last_cmd):
    """frames: list of binned scans, newest first (padded by repeating the oldest)."""
    frames = list(frames)[:FRAMES]
    while len(frames) < FRAMES:
        frames.append(frames[-1])
    return np.concatenate([np.concatenate(frames), np.asarray(goal, np.float32),
                           np.asarray([vel[0] / V_MAX, vel[1] / W_MAX], np.float32),
                           np.asarray([last_cmd[0] / V_MAX, last_cmd[1] / W_MAX], np.float32)]
                          ).astype(np.float32)


def to_command(action):
    a = np.clip(action, -1.0, 1.0)
    v = V_MIN + (a[0] + 1.0) * 0.5 * (V_MAX - V_MIN)
    return float(v), float(a[1] * W_MAX)


def from_command(v, w):
    return np.array([2.0 * (v - V_MIN) / (V_MAX - V_MIN) - 1.0, w / W_MAX], np.float32)


def rewards(pos, goal, min_scan, status):
    """Per-step rewards for one episode.

    pos: (T+1, 2) true robot positions at each observation; min_scan: (T+1,) nearest lidar
    return in metres; status: how the episode ended. Returns (rewards (T,), terminal flag).
    """
    d = np.linalg.norm(pos - np.asarray(goal), axis=1)
    r = PROGRESS * (d[:-1] - d[1:]) - STEP_COST
    r -= CLOSE_COST * np.clip((CLOSE_DIST - min_scan[1:]) / CLOSE_DIST, 0.0, 1.0)
    terminal = False
    if len(r) and status == 'succeeded':
        r[-1] += SUCCESS_REWARD
        terminal = True
    elif len(r) and status == 'collided':
        r[-1] -= COLLISION_COST
        terminal = True
    return r.astype(np.float32), terminal


class NumpyActor:
    """Squashed-Gaussian MLP actor exported by the learner (keys w0, b0, ..., mu_w, mu_b, ls_w, ls_b)."""

    def __init__(self, path):
        data = np.load(path)
        self.version = int(data['version']) if 'version' in data else -1
        self.frame_stride = int(data['frame_stride']) if 'frame_stride' in data else 1
        self.hidden = []
        i = 0
        while f'w{i}' in data:
            self.hidden.append((data[f'w{i}'], data[f'b{i}']))
            i += 1
        self.mu = (data['mu_w'], data['mu_b'])
        self.ls = (data['ls_w'], data['ls_b'])

    def act(self, obs, rng=None):
        h = obs
        for w, b in self.hidden:
            h = np.maximum(h @ w + b, 0.0)
        mu = h @ self.mu[0] + self.mu[1]
        if rng is None:
            return np.tanh(mu)
        log_std = np.clip(h @ self.ls[0] + self.ls[1], -5.0, 2.0)
        return np.tanh(mu + np.exp(log_std) * rng.standard_normal(mu.shape))


def heuristic_action(goal, min_front, rng, noise=0.4):
    """Head for the goal and slow down near obstacles, plus noise; seeds the replay buffer
    before the first trained policy exists."""
    bearing = math.atan2(goal[2], goal[1])
    v = V_MAX * max(0.0, math.cos(bearing)) * min(1.0, max(0.0, (min_front - 0.4) / 1.5))
    w = float(np.clip(2.0 * bearing, -W_MAX, W_MAX))
    return np.clip(from_command(v, w) + noise * rng.standard_normal(2), -1.0, 1.0)
