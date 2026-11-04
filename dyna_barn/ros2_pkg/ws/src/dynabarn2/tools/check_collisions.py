#!/usr/bin/env python3
"""Check that every 'collided' trial really ended next to something.

Recomputes where each obstacle was at the trial's end time, from the world's waypoints, and reports
the free gap between the robot's centre and the nearest obstacle or wall surface. A real collision
leaves a gap no bigger than the Jackal's half diagonal (about 0.35 m) plus a little latency.

Usage: check_collisions.py results.csv worlds_dir
"""
import csv, math, os, re, statistics, sys
import xml.etree.ElementTree as ET

WALLS = [(-10.25, -9.75, -10, 10), (-10, 10, 9.75, 10.25), (-10, 10, -10.25, -9.75)]


def spline_pos(wp, t):
    """gz::math::Spline with auto tangents (Catmull-Rom), as ObstacleMover uses by default."""
    i = next(k for k in range(len(wp) - 1) if t < wp[k + 1][0])
    p = lambda k: wp[k][1:3]
    def tan(k):
        lo, hi = max(k - 1, 0), min(k + 1, len(wp) - 1)
        return [0.5 * (p(hi)[a] - p(lo)[a]) for a in (0, 1)]
    s = (t - wp[i][0]) / (wp[i + 1][0] - wp[i][0])
    h = (2*s**3 - 3*s**2 + 1, s**3 - 2*s**2 + s, -2*s**3 + 3*s**2, s**3 - s**2)
    t0, t1 = tan(i), tan(i + 1)
    return [h[0]*p(i)[a] + h[1]*t0[a] + h[2]*p(i + 1)[a] + h[3]*t1[a] for a in (0, 1)]


def linear_pos(wp, t):
    i = next((k for k in range(len(wp) - 1) if t < wp[k + 1][0]), len(wp) - 2)
    s = min(1.0, (t - wp[i][0]) / (wp[i + 1][0] - wp[i][0]))
    return [wp[i][a] + s * (wp[i + 1][a] - wp[i][a]) for a in (1, 2)]


def obstacles_at(world_file, t):
    found = []
    for model in ET.parse(world_file).getroot().iter('model'):
        plugin = next((p for p in model.findall('plugin') if 'ObstacleMover' in p.get('filename')), None)
        link = model.find('link/collision/geometry')
        if plugin is None:
            continue
        radius = float(link.findtext('cylinder/radius') or float(link.findtext('box/size').split()[0]) / 2 ** .5)
        wp = [tuple(map(float, w.text.split()[:3])) for w in plugin.findall('waypoint')]
        loop = plugin.findtext('loop') == 'true'
        tt = math.fmod(t, wp[-1][0]) if loop else min(t, wp[-1][0] - 1e-6)
        pos = (linear_pos if plugin.findtext('interpolation') == 'linear' else spline_pos)(wp, tt)
        found.append((model.get('name'), pos[0], pos[1], radius))
    return found


def main():
    rows = [r for r in csv.DictReader(open(sys.argv[1])) if r['status'] == 'collided']
    gaps = []
    for r in rows:
        x, y, t = float(r['final_x']), float(r['final_y']), float(r['end_sim_time'])
        name = r['world'].split('/', 1)[1] if r['world'].count('/') else r['world']
        obs = obstacles_at(os.path.join(sys.argv[2], name + '.sdf'), t)
        best, who = min((math.hypot(x - ox, y - oy) - rad, n) for n, ox, oy, rad in obs)
        for x0, x1, y0, y1 in WALLS:
            d = math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1))
            if d < best:
                best, who = d, 'wall'
        gaps.append((best, r['world'], r['planner'], who, r['time_s']))
    v = sorted(g[0] for g in gaps)
    print(f'{len(v)} collisions: gap median {statistics.median(v):.3f} m, max {v[-1]:.3f} m')
    print(f'gap > 0.6 m (suspicious): {sum(g > 0.6 for g in v)}')
    print('hit by wall:', sum(g[3] == 'wall' for g in gaps))
    for g in sorted(gaps, reverse=True)[:6]:
        print(' ', g)


if __name__ == '__main__':
    main()
