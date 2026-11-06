#!/usr/bin/env python3
"""Could floor effects alone explain low rank agreement between fixed and block motion?

Null model: block keeps fixed motion's world order exactly, and only compresses success towards zero.
Each world's fixed success is mapped, in order, onto the observed block success values. Outcomes are
then sampled per repeat, and the Spearman of the two simulated modes is compared with the real one.

Usage: tie_check.py <dynabarn2 package dir> [block tag]
"""
import random
import sys

from analyze_results import spearman
from analyze_sync import PLANNERS, load, success

SIMULATIONS = 2000


def main():
    pkg = sys.argv[1]
    tag = sys.argv[2] if len(sys.argv) > 2 else 'sync_block'
    rng = random.Random(2026)
    fixed, block = load(pkg, 'sync_fixed'), load(pkg, tag)
    repeats = max(int(r['repeat']) for r in block)
    print(f'{tag}, {repeats} repeats\n')
    print('| Planner | Real Spearman | Null mean (95% range) | P(null <= real) | Worlds passing only in '
          'block | Worlds passing in both | Spearman, worlds passing somewhere |')
    print('| --- | --- | --- | --- | --- | --- | --- |')
    for p in PLANNERS:
        a, b = success(fixed, p), success(block, p)
        worlds = sorted(set(a) & set(b))
        real = spearman([a[w] for w in worlds], [b[w] for w in worlds])
        # Break ties at random so the mapping does not depend on world order.
        order = sorted(worlds, key=lambda w: (a[w], rng.random()))
        target = dict(zip(order, sorted(b[w] for w in worlds)))
        null = []
        for _ in range(SIMULATIONS):
            sa = [sum(rng.random() < a[w] for _ in range(3)) / 3 for w in worlds]
            sb = [sum(rng.random() < target[w] for _ in range(repeats)) / repeats for w in worlds]
            rho = spearman(sa, sb)
            if rho == rho:
                null.append(rho)
        null.sort()
        tail = sum(r <= real for r in null) / len(null)
        only = sum(a[w] == 0 and b[w] > 0 for w in worlds)
        both = sum(a[w] > 0 and b[w] > 0 for w in worlds)
        some = [w for w in worlds if a[w] > 0 or b[w] > 0]
        rho_some = spearman([a[w] for w in some], [b[w] for w in some])
        print(f'| {p} | {real:.2f} | {sum(null) / len(null):.2f} ({null[int(.025 * len(null))]:.2f} to '
              f'{null[int(.975 * len(null))]:.2f}) | {tail:.3f} | {only} | {both} | {rho_some:.2f} ({len(some)}) |')


if __name__ == '__main__':
    main()
