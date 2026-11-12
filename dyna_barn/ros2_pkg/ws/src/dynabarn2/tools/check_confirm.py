#!/usr/bin/env python3
"""Score the pre-registered predictions of FINDINGS section 5f on the confirmation world set.

Usage: check_confirm.py <dynabarn2 package dir>
"""
import csv
import json
import os
import random
import sys
from collections import defaultdict

from analyze_results import spearman

PREDICTED, TOLERANCE = 0.228, 0.08


def success(pkg, tag):
    values = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(pkg, 'results', f'{tag}.csv'))):
        if r['status'] != 'no_start' and r['planner'] in ('mppi', 'rpp'):
            values[r['world'].split('/')[1]].append(float(r['status'] == 'succeeded'))
    return {w: sum(v) / len(v) for w, v in values.items()}


def boot(worlds, x, y, rng, n=2000):
    out = []
    for _ in range(n):
        pick = rng.choices(worlds, k=len(worlds))
        r = spearman([x[w] for w in pick], [y[w] for w in pick])
        if r == r:
            out.append(r)
    out.sort()
    return out[int(0.025 * len(out))], out[int(0.975 * len(out))]


def main():
    pkg = sys.argv[1]
    rng = random.Random(2028)
    wdir = os.path.join(pkg, 'worlds', 'dynabarn2_confirm')
    feat = {r['world']: r for r in csv.DictReader(open(os.path.join(wdir, 'sync_features.csv')))}
    man = {e['world']: e for e in json.load(open(os.path.join(wdir, 'manifest.json')))['worlds']}
    block = success(pkg, 'confirm_block')
    fixed = success(pkg, 'confirm_fixed') if os.path.exists(
        os.path.join(pkg, 'results', 'confirm_fixed.csv')) else {}
    worlds = sorted(block)
    hit = {w: float(feat[w]['hit_block_1.0']) for w in worlds}
    blind = {w: man[w]['blind_collision'] for w in worlds}

    rho, (lo, hi) = spearman([hit[w] for w in worlds], [block[w] for w in worlds]), boot(worlds, hit, block, rng)
    rho_b = spearman([blind[w] for w in worlds], [block[w] for w in worlds])
    ok1 = hi < 0 and rho < rho_b
    print(f'Worlds: {len(worlds)}')
    print(f'1. Spearman hit_block_1.0 vs block success {rho:+.2f} ({lo:+.2f} to {hi:+.2f}); '
          f'blind_collision {rho_b:+.2f} -> {"PASS" if ok1 else "FAIL"}')

    safe = [block[w] for w in worlds if hit[w] == 0]
    rest = [block[w] for w in worlds if hit[w] == 1]
    ms, mr = sum(safe) / len(safe), sum(rest) / len(rest)
    print(f'2. Block-safe worlds {ms:.2f} (n={len(safe)}), others {mr:.2f} (n={len(rest)}) -> '
          f'{"PASS" if ms >= 0.8 and mr <= 0.2 else "FAIL"}')

    total = sum(block.values()) / len(block)
    print(f'3. Set block success {total:.3f} vs predicted {PREDICTED} +/- {TOLERANCE} -> '
          f'{"PASS" if abs(total - PREDICTED) <= TOLERANCE else "FAIL"}')

    if fixed:
        common = sorted(set(fixed) & set(block))
        f = sum(fixed[w] for w in common) / len(common)
        b = sum(block[w] for w in common) / len(common)
        print(f'Not pre-registered: fixed {f:.3f}, block {b:.3f}, drop {b - f:+.3f} over {len(common)} worlds')
        hf = {w: float(feat[w]['hit_fixed_1.0']) for w in common}
        print(f'  Spearman hit_fixed_1.0 vs fixed success {spearman([hf[w] for w in common], [fixed[w] for w in common]):+.2f}, '
              f'blind_collision {spearman([blind[w] for w in common], [fixed[w] for w in common]):+.2f}')


if __name__ == '__main__':
    main()
