#!/usr/bin/env python3
"""Summarise baseline results for the paper. Standard library only.

Reads results/<tag>/*.csv for the original worlds and for DynaBARN II in fixed, yield and block
modes, and prints Markdown tables. Worlds are the unit of analysis: repeats of one world are averaged
first, and confidence intervals come from resampling worlds (bootstrap, fixed seed).

Usage: analyze_results.py <dynabarn2 package dir>
"""
import csv
import glob
import json
import os
import random
import statistics
import sys
from collections import Counter, defaultdict

BOOTSTRAP = 2000
LEVELS = ('easy', 'medium', 'hard')
PLANNERS = ('dwb', 'mppi')
VOID = ('no_start',)  # startup failures: the planner never ran, so they say nothing about it


def load(pkg, tag):
    rows = []
    for path in glob.glob(os.path.join(pkg, 'results', tag, '*.csv')):
        rows += csv.DictReader(open(path))
    return rows


def by_world(rows, planner, field='success'):
    """{world: mean over repeats} for one planner."""
    values = defaultdict(list)
    for r in rows:
        if r['planner'] == planner and r['status'] not in VOID:
            values[r['world']].append(float(r['status'] == 'succeeded') if field == 'success'
                                      else float(r['score']))
    return {w: sum(v) / len(v) for w, v in values.items()}


def ci(values, rng):
    means = sorted(sum(rng.choices(values, k=len(values))) / len(values)
                   for _ in range(BOOTSTRAP))
    return means[int(0.025 * BOOTSTRAP)], means[int(0.975 * BOOTSTRAP)]


def spearman(a, b):
    def ranks(x):
        order = sorted(range(len(x)), key=lambda i: x[i])
        out = [0.0] * len(x)
        i = 0
        while i < len(x):
            j = i
            while j + 1 < len(x) and x[order[j + 1]] == x[order[i]]:
                j += 1
            for k in range(i, j + 1):
                out[order[k]] = (i + j) / 2
            i = j + 1
        return out
    ra, rb = ranks(a), ranks(b)
    ma, mb = statistics.mean(ra), statistics.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** 0.5
    return num / den if den else float('nan')


def status_table(name, rows, rng):
    print(f'\n### {name}\n')
    print('| Planner | Worlds | Trials | Success (95% CI) | Score | Collided | Timeout | Hit at start '
          '| Void |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for p in PLANNERS:
        trials = [r for r in rows if r['planner'] == p]
        if not trials:
            continue
        c = Counter(r['status'] for r in trials)
        ok = list(by_world(rows, p).values())
        score = list(by_world(rows, p, 'score').values())
        lo, hi = ci(ok, rng)
        print(f'| {p} | {len(ok)} | {len(trials)} | {sum(ok) / len(ok):.2f} ({lo:.2f}-{hi:.2f}) | '
              f'{sum(score) / len(score):.3f} | {c["collided"]} | {c["timeout"]} | '
              f'{c["hit_at_start"]} | {c["no_start"]} |')


def group_table(name, rows, group_of, groups):
    print(f'\n### {name}\n')
    print('| Group | ' + ' | '.join(PLANNERS) + ' |')
    print('| --- |' + ' --- |' * len(PLANNERS))
    for g in groups:
        cells = []
        for p in PLANNERS:
            values = [v for w, v in by_world(rows, p).items() if group_of(w) == g]
            cells.append(f'{sum(values) / len(values):.2f} ({len(values)})' if values else '-')
        print(f'| {g} | ' + ' | '.join(cells) + ' |')


def original_group(world):
    i = int(world.rsplit('_', 1)[1])
    return ('0-19 easy, few' if i < 20 else '20-29 hard, few' if i < 30
            else '30-39 easy, dense' if i < 40 else '40-59 hard, dense')


def paired(base, other, label, planner, rng):
    a, b = by_world(base, planner), by_world(other, planner)
    worlds = sorted(set(a) & set(b))
    if not worlds:
        return
    diffs = [b[w] - a[w] for w in worlds]
    lo, hi = ci(diffs, rng)
    print(f'| {planner} | {label} | {len(worlds)} | {sum(a[w] for w in worlds) / len(worlds):.2f} | '
          f'{sum(b[w] for w in worlds) / len(worlds):.2f} | {sum(diffs) / len(diffs):+.3f} '
          f'({lo:+.3f} to {hi:+.3f}) | {sum(d > 0 for d in diffs)}/{sum(d < 0 for d in diffs)} |')


def rank_gap(rows, rng):
    """DWB minus MPPI success, paired by world: its sign is the planners' ranking."""
    a, b = by_world(rows, 'dwb'), by_world(rows, 'mppi')
    worlds = sorted(set(a) & set(b))
    if not worlds:
        return None
    diffs = [a[w] - b[w] for w in worlds]
    return (sum(diffs) / len(diffs), *ci(diffs, rng), len(worlds))


def main():
    pkg = sys.argv[1]
    rng = random.Random(2026)
    original = load(pkg, 'dynabarn')
    fixed = load(pkg, 'dynabarn2')
    yielding = load(pkg, 'dynabarn2_yield')
    blocking = load(pkg, 'dynabarn2_block')
    with open(os.path.join(pkg, 'worlds', 'dynabarn2', 'manifest.json')) as f:
        info = {e['world']: e for e in json.load(f)['worlds']}
    entry = lambda world: info[world.split('/')[1]]  # noqa: E731

    status_table('Original 60 worlds', original, rng)
    group_table('Success by original world group (worlds)', original, original_group,
                ['0-19 easy, few', '20-29 hard, few', '30-39 easy, dense', '40-59 hard, dense'])
    status_table('DynaBARN II, fixed motion', fixed, rng)
    group_table('Success by level (worlds)', fixed, lambda w: entry(w)['level'], LEVELS)
    group_table('Success by family (worlds)', fixed, lambda w: entry(w)['family'],
                ['crowd', 'lanes', 'traffic', 'stop_and_go', 'mixed'])

    print('\n### Does blind_collision predict success? (DynaBARN II, fixed motion)\n')
    print('| Planner | Worlds | Spearman, blind_collision vs success |')
    print('| --- | --- | --- |')
    for p in PLANNERS:
        ok = by_world(fixed, p)
        worlds = sorted(ok)
        if worlds:
            rho = spearman([entry(w)['blind_collision'] for w in worlds], [ok[w] for w in worlds])
            print(f'| {p} | {len(worlds)} | {rho:.2f} |')

    if yielding or blocking:
        status_table('DynaBARN II, obstacles yield', yielding, rng)
        status_table('DynaBARN II, obstacles block', blocking, rng)
        print('\n### Replay vs reactive, paired by world\n')
        print('| Planner | Change | Worlds | Fixed success | Reactive success | Mean difference '
              '(95% CI) | Worlds up/down |')
        print('| --- | --- | --- | --- | --- | --- | --- |')
        for p in PLANNERS:
            paired(fixed, yielding, 'yield', p, rng)
            paired(fixed, blocking, 'block', p, rng)
        print('\n### Does the DWB vs MPPI ranking change?\n')
        print('| Mode | Worlds | DWB minus MPPI success (95% CI) |')
        print('| --- | --- | --- |')
        for name, rows in (('fixed', fixed), ('yield', yielding), ('block', blocking)):
            gap = rank_gap(rows, rng)
            if gap:
                print(f'| {name} | {gap[3]} | {gap[0]:+.3f} ({gap[1]:+.3f} to {gap[2]:+.3f}) |')
        for level in LEVELS:
            for name, rows in (('fixed', fixed), ('yield', yielding), ('block', blocking)):
                subset = [r for r in rows if entry(r['world'])['level'] == level]
                gap = rank_gap(subset, rng)
                if gap:
                    print(f'| {level}, {name} | {gap[3]} | {gap[0]:+.3f} '
                          f'({gap[1]:+.3f} to {gap[2]:+.3f}) |')


if __name__ == '__main__':
    main()
