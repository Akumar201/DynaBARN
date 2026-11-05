#!/usr/bin/env python3
"""Summarise the synchronised-start experiments. Standard library only.

Worlds are the unit of analysis: repeats are averaged first, and confidence intervals resample worlds
(bootstrap, fixed seed). Trials that never started are excluded.

Usage: analyze_sync.py <dynabarn2 package dir>
"""
import csv
import glob
import itertools
import json
import math
import os
import random
import statistics
import sys
from collections import Counter, defaultdict

from analyze_results import BOOTSTRAP, LEVELS, ci, spearman

PLANNERS = ('dwb', 'mppi', 'rpp')
MODES = (('fixed', 'sync_fixed'), ('yield 0.5', 'sync_yield_s05'), ('yield 1.0', 'sync_yield'),
         ('yield 2.0', 'sync_yield_s20'), ('block 0.5', 'sync_block_s05'),
         ('block 1.0', 'sync_block'), ('block 2.0', 'sync_block_s20'))
GOAL = (-9.0, 0.0)
STALL_RADIUS = 3.0  # m: a timeout this close to the goal is odometry drift, not failure to get there


def load(pkg, tag):
    rows = []
    for path in glob.glob(os.path.join(pkg, 'results', tag, '*.csv')):
        repeat = os.path.basename(path)[:-4].rsplit('_', 1)[1]
        rows += [dict(r, repeat=repeat) for r in csv.DictReader(open(path))
                 if r['status'] != 'no_start']
    return rows


def stalled(row):
    return row['status'] == 'timeout' and math.dist(
        (float(row['final_x']), float(row['final_y'])), GOAL) <= STALL_RADIUS


def success(rows, planner, keep=lambda world: True, count_stalls=False):
    values = defaultdict(list)
    for r in rows:
        if r['planner'] == planner and keep(r['world']):
            values[r['world']].append(float(r['status'] == 'succeeded'
                                            or (count_stalls and stalled(r))))
    return {w: sum(v) / len(v) for w, v in values.items()}


def mean(values):
    return sum(values) / len(values)


def agreement(a, b, rng):
    worlds = sorted(set(a) & set(b))
    rho = spearman([a[w] for w in worlds], [b[w] for w in worlds])
    samples = []
    for _ in range(BOOTSTRAP):
        pick = rng.choices(worlds, k=len(worlds))
        r = spearman([a[w] for w in pick], [b[w] for w in pick])
        if r == r:
            samples.append(r)
    samples.sort()
    return rho, samples[int(0.025 * len(samples))], samples[int(0.975 * len(samples))]


def repeatability(rows):
    """Share of world/planner pairs with mixed outcomes, and the spread of finish times within them."""
    groups = defaultdict(list)
    for r in rows:
        if r['planner'] in ('dwb', 'mppi'):  # the planners both sets of runs share
            groups[(r['world'], r['planner'])].append(r)
    groups = {k: v for k, v in groups.items() if len(v) == 3}
    mixed = sum(len({r['status'] for r in v}) > 1 for v in groups.values())
    spread = [max(float(r['time_s']) for r in v) - min(float(r['time_s']) for r in v)
              for v in groups.values() if len({r['status'] for r in v}) == 1]
    return mixed, len(groups), statistics.median(spread)


def main():
    pkg = sys.argv[1]
    rng = random.Random(2026)
    data = {name: load(pkg, tag) for name, tag in MODES}
    with open(os.path.join(pkg, 'worlds', 'dynabarn2', 'manifest.json')) as f:
        info = {e['world']: e for e in json.load(f)['worlds']}
    level = lambda w: info[w.split('/')[1]]['level']  # noqa: E731

    print('### Repeatability over 3 repeats, DWB and MPPI\n')
    print('| Runs | Mixed pairs | Median finish-time spread, unmixed pairs (s) |')
    print('| --- | --- | --- |')
    for name, tag in (('unsynced fixed', 'dynabarn2'), ('unsynced yield', 'dynabarn2_yield'),
                      ('unsynced block', 'dynabarn2_block'), ('unsynced original', 'dynabarn'),
                      ('synced fixed', 'sync_fixed'), ('synced yield', 'sync_yield'),
                      ('synced block', 'sync_block'), ('synced original', 'sync_original')):
        m, n, s = repeatability(load(pkg, tag))
        print(f'| {name} | {m}/{n} | {s:.2f} |')

    print('\n### Success by mode\n')
    print('| Mode | Trials | ' + ' | '.join(PLANNERS) + ' | Timeouts |')
    print('| --- | --- |' + ' --- |' * len(PLANNERS) + ' --- |')
    for name, rows in list(data.items()) + [('original, fixed', load(pkg, 'sync_original'))]:
        cells = []
        for p in PLANNERS:
            ok = list(success(rows, p).values())
            lo, hi = ci(ok, rng)
            cells.append(f'{mean(ok):.2f} ({lo:.2f}-{hi:.2f})')
        timeouts = Counter(r['status'] for r in rows)['timeout']
        print(f'| {name} | {len(rows)} | ' + ' | '.join(cells) + f' | {timeouts} |')

    print('\n### Success by level, DWB / MPPI / RPP\n')
    print('| Mode | ' + ' | '.join(LEVELS) + ' |')
    print('| --- |' + ' --- |' * len(LEVELS))
    for name, rows in data.items():
        cells = []
        for g in LEVELS:
            cells.append(' / '.join(
                f'{mean(list(success(rows, p, lambda w: level(w) == g).values())):.2f}'
                for p in PLANNERS))
        print(f'| {name} | ' + ' | '.join(cells) + ' |')

    for label, stalls in (('as run', False), ('counting near-goal stalls as arrivals', True)):
        print(f'\n### Paired planner gaps, {label} (first minus second, success)\n')
        print('| Mode | Worlds | Stalls | '
              + ' | '.join(f'{a}-{b}' for a, b in itertools.combinations(PLANNERS, 2)) + ' |')
        print('| --- | --- | --- |' + ' --- |' * 3)
        for name, rows in data.items():
            cells = []
            for a, b in itertools.combinations(PLANNERS, 2):
                sa, sb = success(rows, a, count_stalls=stalls), success(rows, b, count_stalls=stalls)
                worlds = sorted(set(sa) & set(sb))
                diffs = [sa[w] - sb[w] for w in worlds]
                lo, hi = ci(diffs, rng)
                mark = ' *' if lo > 0 or hi < 0 else ''
                cells.append(f'{mean(diffs):+.3f} ({lo:+.3f} to {hi:+.3f}){mark}')
            print(f'| {name} | {len(worlds)} | {sum(stalled(r) for r in rows)} | '
                  + ' | '.join(cells) + ' |')

    print('\n### World difficulty order against fixed motion (Spearman, 95% CI)\n')
    print('| Planner | Mode | Spearman | Repeat vs other two, same mode |')
    print('| --- | --- | --- | --- |')
    for p in PLANNERS:
        for name in ('fixed', 'yield 1.0', 'block 1.0'):
            rows = data[name]
            rhos = []
            for held in '123':
                one = success([r for r in rows if r['repeat'] == held], p)
                rest = success([r for r in rows if r['repeat'] != held], p)
                worlds = sorted(set(one) & set(rest))
                rhos.append(spearman([one[w] for w in worlds], [rest[w] for w in worlds]))
            ceiling = mean([r for r in rhos if r == r])
            if name == 'fixed':
                print(f'| {p} | fixed | - | {ceiling:.2f} |')
                continue
            rho, lo, hi = agreement(success(data['fixed'], p), success(rows, p), rng)
            print(f'| {p} | {name} | {rho:.2f} ({lo:.2f} to {hi:.2f}) | {ceiling:.2f} |')

    print('\n### blind_collision against success (Spearman)\n')
    print('| Mode | ' + ' | '.join(PLANNERS) + ' |')
    print('| --- |' + ' --- |' * len(PLANNERS))
    for name in ('fixed', 'yield 1.0', 'block 1.0'):
        cells = []
        for p in PLANNERS:
            ok = success(data[name], p)
            worlds = sorted(ok)
            key = lambda w: info[w.split('/')[1]]['blind_collision']  # noqa: E731
            cells.append(f'{spearman([key(w) for w in worlds], [ok[w] for w in worlds]):.2f}')
        print(f'| {name} | ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
