#!/usr/bin/env python3
"""Classify how robots get hit in synchronised fixed-motion trials.

For each collision, finds the obstacle at the robot and describes the hit in the world frame, where the
robot is travelling roughly towards -x: where the obstacle was relative to the robot just before, and
whether it was moving towards the robot.

Usage: analyze_hits.py <dynabarn2 package dir> [tag] [worlds subdir]
"""
import csv
import math
import statistics
import sys
from collections import Counter

from check_collisions import obstacles_at

LOOK_BACK = 1.0  # s before the hit


def main():
    pkg = sys.argv[1]
    tag = sys.argv[2] if len(sys.argv) > 2 else 'sync_fixed'
    worlds = sys.argv[3] if len(sys.argv) > 3 else 'dynabarn2'
    for planner in ('dwb', 'mppi', 'rpp', 'mppi_pred'):
        rows = [r for r in csv.DictReader(open(f'{pkg}/results/{tag}.csv'))
                if r['planner'] == planner and r['status'] == 'collided']
        side, speeds, closing = Counter(), [], []
        for r in rows:
            x, y, t = float(r['final_x']), float(r['final_y']), float(r['time_s'])
            path = f"{pkg}/worlds/{r['world'].split('/')[0]}/{r['world'].split('/')[1]}.sdf"
            now = obstacles_at(path, t)
            name = min(now, key=lambda o: math.hypot(o[1] - x, o[2] - y) - o[3])[0]
            hit = next(o for o in now if o[0] == name)
            before = next(o for o in obstacles_at(path, max(t - LOOK_BACK, 0.0)) if o[0] == name)
            vx, vy = (hit[1] - before[1]) / LOOK_BACK, (hit[2] - before[2]) / LOOK_BACK
            # Bearing of the obstacle from the robot, measured against travel direction (-x).
            bearing = abs(math.degrees(math.atan2(before[2] - y, -(before[1] - x))))
            side['front' if bearing < 45 else 'side' if bearing < 135 else 'rear'] += 1
            speeds.append(math.hypot(vx, vy))
            dx, dy = x - before[1], y - before[2]
            norm = math.hypot(dx, dy) or 1.0
            closing.append((vx * dx + vy * dy) / norm)
        n = len(rows) or 1
        print(f'{planner}: {len(rows)} hits, front {side["front"] / n:.2f} side {side["side"] / n:.2f} '
              f'rear {side["rear"] / n:.2f}; obstacle speed median {statistics.median(speeds):.2f} m/s, '
              f'moving towards robot in {sum(c > 0.1 for c in closing) / n:.2f} of hits, '
              f'median time {statistics.median(float(r["time_s"]) for r in rows):.1f} s')


if __name__ == '__main__':
    main()
