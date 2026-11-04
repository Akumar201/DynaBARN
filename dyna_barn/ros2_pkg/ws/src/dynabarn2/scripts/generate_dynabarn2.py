#!/usr/bin/env python3
"""Build the fixed DynaBARN II world set.

Every family/level pair gets --per_cell worlds. Obstacles bounce off each other (see
scenario_sim.py); a world is kept only if a robot can actually reach the goal, and its measured
difficulty is recorded in manifest.json. The same --seed always produces the same set.

  crowd        discs wander and bounce
  lanes        two-way lanes
  traffic      lanes with lane changes
  stop_and_go  lane traffic where obstacles keep stopping and starting
  mixed        mixed sizes and shapes moving through static clutter

Example:
  generate_dynabarn2.py --save_dir worlds/dynabarn2 --per_cell 4
"""
import argparse
import json
import os

import numpy as np

import scenario_sim as sim
import world_sdf

DURATION = 180.0  # s, longer than any trial (startup plus the 100 s time limit)
LEVELS = {        # number of obstacles, speed range (m/s), radius range (m), static discs
    'easy':   dict(count=6,  speed=(0.4, 0.8), radius=(0.3, 0.35), statics=4),
    'medium': dict(count=12, speed=(0.7, 1.2), radius=(0.3, 0.4),  statics=8),
    'hard':   dict(count=20, speed=(1.0, 1.6), radius=(0.3, 0.45), statics=12),
}
FAMILIES = ('crowd', 'lanes', 'traffic', 'stop_and_go', 'mixed')


def place_statics(rng, count):
    """Static discs, kept clear of the start side and of a 1 m circle around the goal."""
    statics = []
    while len(statics) < count:
        x, y, r = rng.uniform(-7.5, 7.5), rng.uniform(-7.5, 7.5), rng.uniform(0.2, 0.5)
        if all(np.hypot(x - a, y - b) > r + c + 1.0 for a, b, c in statics):
            statics.append((x, y, r))
    return statics


def make_scenario(family, level, rng):
    cfg = LEVELS[level]
    n = cfg['count']
    speeds = rng.uniform(*cfg['speed'], n)
    if family == 'mixed':
        radii = rng.uniform(0.2, 0.7, n)
        return sim.Scenario('crowd', radii, speeds, place_statics(rng, cfg['statics']), rng)
    radii = rng.uniform(*cfg['radius'], n)
    if family == 'crowd':
        return sim.Scenario('crowd', radii, speeds, rng=rng)
    axis = str(rng.choice(['row', 'column']))
    if family == 'lanes':
        return sim.Scenario('lanes', radii, speeds, rng=rng, axis=axis)
    if family == 'traffic':
        return sim.Scenario('traffic', radii, speeds, rng=rng, axis=axis, lane_change_prob=0.08)
    return sim.Scenario('traffic', radii, speeds, rng=rng, axis=axis, lane_change_prob=0.04,
                        stop_and_go=True)


def build_world(sc, times, traj, rng, family):
    models = []
    for i in range(traj.shape[1]):
        keep = sim.simplify(times, traj[:, i])
        # Mixed worlds also vary the shape; boxes fit inside the disc used for collisions.
        shape = 'box' if family == 'mixed' and rng.random() < 0.4 else 'cylinder'
        waypoints = [(float(times[k]), float(traj[k, i, 0]), float(traj[k, i, 1])) for k in keep]
        models.append(world_sdf.obstacle(f'obs_{i}', waypoints, radius=float(sc.radii[i]),
                                         interpolation='linear', loop=False, shape=shape))
    for i, (x, y, r) in enumerate(sc.statics):
        models.append(world_sdf.static_obstacle(f'static_{i}', x, y, r))
    return world_sdf.world(models)


def stats(sc, times, traj):
    step = np.diff(traj, axis=0) / np.diff(times)[:, None, None]
    speed = np.linalg.norm(step, axis=2)
    gap = np.inf
    for i in range(traj.shape[1] - 1):
        d = np.linalg.norm(traj[:, i + 1:] - traj[:, i:i + 1], axis=2)
        gap = min(gap, float((d - sc.radii[i + 1:] - sc.radii[i]).min()))
    return {'peak_speed': round(float(speed.max()), 3),
            'mean_speed': round(float(speed.mean()), 3),
            'min_gap': round(gap, 4)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--save_dir', default='./worlds/dynabarn2')
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--per_cell', type=int, default=4, help='worlds per family and level')
    ap.add_argument('--families', nargs='+', choices=FAMILIES, default=list(FAMILIES))
    ap.add_argument('--levels', nargs='+', choices=list(LEVELS), default=list(LEVELS))
    args = ap.parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    manifest, index = [], 0
    for family in args.families:
        for level in args.levels:
            made = 0
            attempt = 0
            while made < args.per_cell:
                seed = int(np.random.SeedSequence(
                    [args.seed, FAMILIES.index(family), list(LEVELS).index(level), attempt]
                ).generate_state(1)[0])
                attempt += 1
                rng = np.random.default_rng(seed)
                sc = make_scenario(family, level, rng)
                times, traj = sim.simulate(sc, DURATION)
                best = sim.earliest_arrival(times, traj, sc.radii, sc.statics)
                if best is None:
                    print(f'  rejected {family}/{level} seed {seed}: no route to the goal')
                    continue
                entry = {'world': f'world_{index}', 'family': family, 'level': level, 'seed': seed,
                         'obstacles': len(sc.radii), 'statics': len(sc.statics),
                         'earliest_arrival_s': round(best, 2),
                         'blind_collision': sim.blind_collision(times, traj, sc.radii, sc.statics),
                         **stats(sc, times, traj)}
                with open(os.path.join(args.save_dir, f'world_{index}.sdf'), 'w') as f:
                    f.write(build_world(sc, times, traj, rng, family))
                manifest.append(entry)
                print(entry)
                index += 1
                made += 1

    with open(os.path.join(args.save_dir, 'manifest.json'), 'w') as f:
        json.dump({'seed': args.seed, 'worlds': manifest}, f, indent=1)
    print(f'Wrote {index} worlds to {args.save_dir}')


if __name__ == '__main__':
    main()
