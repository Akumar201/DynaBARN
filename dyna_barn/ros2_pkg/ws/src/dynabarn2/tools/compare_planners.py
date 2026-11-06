#!/usr/bin/env python3
"""Paired success comparison of one planner against plain MPPI, per synchronised mode.

Usage: compare_planners.py <dynabarn2 package dir> <planner> [tag ...]
"""
import collections
import csv
import glob
import json
import os
import random
import statistics
import sys

from analyze_results import ci
from analyze_sync import load, mean, success


def main():
    pkg, planner = sys.argv[1], sys.argv[2]
    tags = sys.argv[3:] or ['sync_fixed', 'sync_yield', 'sync_block']
    rng = random.Random(2026)
    with open(os.path.join(pkg, 'worlds', 'dynabarn2', 'manifest.json')) as f:
        info = {e['world']: e for e in json.load(f)['worlds']}
    for tag in tags:
        files = glob.glob(os.path.join(pkg, 'results', tag, f'*_{planner}_*.csv'))
        raw = [r for f in files for r in csv.DictReader(open(f))]
        if not raw:
            continue
        rows = load(pkg, tag)
        print(f'{tag}: {len(raw)} trials', dict(collections.Counter(r['status'] for r in raw)))
        for label, stalls in (('as run', False), ('stalls as arrivals', True)):
            a = success(rows, 'mppi', count_stalls=stalls)
            b = success(rows, planner, count_stalls=stalls)
            worlds = sorted(set(a) & set(b))
            diffs = [b[w] - a[w] for w in worlds]
            lo, hi = ci(diffs, rng)
            print(f'  {label}: mppi {mean([a[w] for w in worlds]):.3f}, {planner} '
                  f'{mean([b[w] for w in worlds]):.3f}, diff {mean(diffs):+.3f} ({lo:+.3f} to {hi:+.3f}), '
                  f'worlds up/down {sum(d > 0 for d in diffs)}/{sum(d < 0 for d in diffs)}, n={len(worlds)}')
        for level in ('easy', 'medium', 'hard'):
            keep = lambda w: info[w.split('/')[1]]['level'] == level  # noqa: E731,B023
            a, b = success(rows, 'mppi', keep), success(rows, planner, keep)
            worlds = sorted(set(a) & set(b))
            print(f'  {level}: mppi {mean([a[w] for w in worlds]):.2f}, {planner} '
                  f'{mean([b[w] for w in worlds]):.2f}')
        times = {p: [float(r['time_s']) for r in rows if r['planner'] == p and r['status'] == 'succeeded']
                 for p in ('mppi', planner)}
        print(f'  median success time: mppi {statistics.median(times["mppi"]):.1f} s, {planner} '
              f'{statistics.median(times[planner]):.1f} s')


if __name__ == '__main__':
    main()
