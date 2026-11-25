#!/usr/bin/env python3
"""Paired world-level comparison of the learned planner against the baselines of another results file.

Per world, success and score are averaged over repeats; the difference learned minus baseline is averaged over
the worlds both have, with a 95% bootstrap interval that resamples worlds (no_start trials are dropped).

Usage: compare_learned.py <learned.csv> <baselines.csv>
"""
import csv
import random
import sys
from collections import defaultdict


def per_world(path):
    out = defaultdict(lambda: defaultdict(list))   # planner -> world -> [(success, score)]
    for r in csv.DictReader(open(path)):
        if r['status'] != 'no_start':
            out[r['planner']][r['world']].append((float(r['status'] == 'succeeded'), float(r['score'])))
    return {p: {w: (sum(s for s, _ in v) / len(v), sum(c for _, c in v) / len(v)) for w, v in ws.items()}
            for p, ws in out.items()}


def boot(diffs, rng, n=4000):
    means = sorted(sum(rng.choices(diffs, k=len(diffs))) / len(diffs) for _ in range(n))
    return means[int(0.025 * n)], means[int(0.975 * n)]


def main():
    learned = per_world(sys.argv[1])['learned']
    base = per_world(sys.argv[2])
    rng = random.Random(2026)
    print(f'{"planner":<12} {"worlds":>6} {"success":>8} {"score":>7}   learned minus planner: success, score (95% CI)')
    lw = sorted(learned)
    print(f'{"learned":<12} {len(lw):>6} {sum(learned[w][0] for w in lw) / len(lw):>8.3f} '
          f'{sum(learned[w][1] for w in lw) / len(lw):>7.3f}')
    for planner, ws in sorted(base.items()):
        common = sorted(set(ws) & set(learned))
        ds = [learned[w][0] - ws[w][0] for w in common]
        dc = [learned[w][1] - ws[w][1] for w in common]
        (a, b), (c, d) = boot(ds, rng), boot(dc, rng)
        print(f'{planner:<12} {len(common):>6} {sum(ws[w][0] for w in common) / len(common):>8.3f} '
              f'{sum(ws[w][1] for w in common) / len(common):>7.3f}   '
              f'{sum(ds) / len(ds):+.3f} ({a:+.3f} to {b:+.3f}), {sum(dc) / len(dc):+.3f} ({c:+.3f} to {d:+.3f})')


if __name__ == '__main__':
    main()
