#!/usr/bin/env python3
"""Difficulty features for the synchronised start, computed from world files.

The generator's manifest metrics average over many robot departure times. With DYNABARN_SYNC_START the
robot always meets the obstacles at trajectory time 0, so only that departure matters. This replays each
world's obstacle trajectories (linear interpolation of the dense waypoints) against a robot driving
straight from start to goal, with obstacles on their fixed paths or reacting as in ObstacleMover.

Usage: sync_features.py <world dir> [<world dir> ...]   -> CSV on stdout
"""
import csv
import glob
import math
import os
import re
import sys

import numpy as np

START, GOAL = np.array([11.0, 0.0]), np.array([-9.0, 0.0])
ROBOT_RADIUS = 0.3
INFLUENCE, RESPONSE, RECOVERY, MAX_OFFSET = 2.5, 1.0, 2.0, 2.0  # ObstacleMover defaults
DT = 0.02
SPEEDS = (0.5, 1.0, 1.5)


def load(path):
    text = open(path).read()
    movers, statics = [], []
    for name, model in re.findall(r'<model name="([^"]*)">(.*?)</model>', text, re.S):
        m = re.search(r'<radius>([\d.]+)</radius>', model)
        if m:
            radius = float(m.group(1))
        else:
            m = re.search(r'<box><size>([\d.]+)', model)
            radius = float(m.group(1)) / math.sqrt(2) if m else 0.5
        wps = re.findall(r'<waypoint>([-\d.]+) ([-\d.]+) ([-\d.]+)', model)
        if wps:
            arr = np.array(wps, dtype=float)
            movers.append((arr[:, 0], arr[:, 1:3], radius))
        elif name.startswith('static_'):
            p = re.search(r'<pose>([-\d.]+) ([-\d.]+)', model)
            if p:
                statics.append((float(p.group(1)), float(p.group(2)), radius))
    return movers, statics


def position(mover, t):
    times, xy, _ = mover
    t = math.fmod(t, times[-1]) if times[-1] > 0 else 0.0
    return np.array([np.interp(t, times, xy[:, 0]), np.interp(t, times, xy[:, 1])])


def drive(movers, statics, speed, sign):
    """Straight-line robot; sign 0 fixed, +1 yield, -1 block. Returns hit time (or None) and exposure."""
    route = np.linalg.norm(GOAL - START)
    offsets = [np.zeros(2) for _ in movers]
    exposure = 0.0
    near = set()
    for step in range(int(route / speed / DT) + 1):
        t = step * DT
        robot = START + (GOAL - START) * min(t * speed / route, 1.0)
        for i, mv in enumerate(movers):
            base = position(mv, t)
            if sign:
                rate = -offsets[i] / RECOVERY
                away = base + offsets[i] - robot
                d = np.linalg.norm(away)
                if 1e-3 < d < INFLUENCE:
                    rate = rate + away / d * sign * RESPONSE * (1 - d / INFLUENCE)
                offsets[i] = offsets[i] + rate * DT
                n = np.linalg.norm(offsets[i])
                if n > MAX_OFFSET:
                    offsets[i] *= MAX_OFFSET / n
            d = np.linalg.norm(base + offsets[i] - robot)
            if d < INFLUENCE:
                exposure += DT
                near.add(i)
            if d < mv[2] + ROBOT_RADIUS:
                return t, exposure, len(near)
        for sx, sy, r in statics:
            if math.hypot(robot[0] - sx, robot[1] - sy) < r + ROBOT_RADIUS:
                return t, exposure, len(near)
    return None, exposure, len(near)


def main():
    out = csv.writer(sys.stdout)
    header = ['set', 'world']
    for v in SPEEDS:
        header += [f'hit_fixed_{v}', f'hit_block_{v}', f'hit_yield_{v}']
    header += ['hit_time_fixed_1.0', 'hit_time_block_1.0', 'exposure_1.0', 'near_count_1.0']
    out.writerow(header)
    for wdir in sys.argv[1:]:
        for path in sorted(glob.glob(os.path.join(wdir, 'world_*.sdf')),
                           key=lambda p: int(re.findall(r'\d+', os.path.basename(p))[0])):
            movers, statics = load(path)
            row = [os.path.basename(os.path.normpath(wdir)), os.path.basename(path)[:-4]]
            extra = None
            for v in SPEEDS:
                fixed = drive(movers, statics, v, 0)
                block = drive(movers, statics, v, -1)
                yld = drive(movers, statics, v, 1)
                row += [int(fixed[0] is not None), int(block[0] is not None), int(yld[0] is not None)]
                if v == 1.0:
                    extra = [fixed[0] if fixed[0] is not None else '', block[0] if block[0] is not None else '',
                             round(fixed[1], 2), fixed[2]]
            out.writerow(row + extra)
            sys.stdout.flush()


if __name__ == '__main__':
    main()
