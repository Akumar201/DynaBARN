#!/usr/bin/env python3
"""Compare obstacle_predictor's moving tracks with the true obstacle motion in synchronised runs.

A track counts as real if it lies within 0.8 m of a true obstacle centre. Track positions are in the
robot's odom frame, converted using the spawn pose (11, 0, facing -x), so they include odometry drift.

Usage: evaluate_tracker.py <worlds dir> <world name> <track log> <robot start sim time>
"""
import math
import statistics
import sys

from check_collisions import obstacles_at


def main():
    worlds, name, log, start = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
    path = f'{worlds}/{name}.sdf'
    real, ghost, speed_errors, heading_errors = 0, 0, [], []
    for line in list(open(log))[::10]:  # every 10th sample: parsing the world for each is slow
        t, x, y, vx, vy = map(float, line.split())
        # Odom starts at (11, 0) rotated by pi.
        wx, wy, wvx, wvy = 11 - x, -y, -vx, -vy
        tt = t - start
        if tt < 0:
            continue
        now = {n: (ox, oy) for n, ox, oy, _ in obstacles_at(path, tt)}
        name_hit, (ox, oy) = min(now.items(), key=lambda kv: math.hypot(kv[1][0] - wx, kv[1][1] - wy))
        if math.hypot(ox - wx, oy - wy) > 0.8:
            ghost += 1
            continue
        real += 1
        later = {n: (a, b) for n, a, b, _ in obstacles_at(path, tt + 0.1)}[name_hit]
        tvx, tvy = (later[0] - ox) / 0.1, (later[1] - oy) / 0.1
        speed_errors.append(math.hypot(wvx - tvx, wvy - tvy))
    total = real + ghost
    print(f'{name}: {total} track samples, real {real / max(total, 1):.2f}, '
          f'median velocity error {statistics.median(speed_errors) if speed_errors else float("nan"):.2f} m/s')


if __name__ == '__main__':
    main()
