#!/usr/bin/env python3
"""Original DynaBARN worlds: obstacles on random polynomial trajectories.

Port of DynaBARN/generate_plugins_gz11.py for Gazebo Harmonic. Builds a pool of
random trajectories, then writes --n_worlds worlds that each sample
--min_object..--max_object obstacles from the pool.

Example:
  generate_polynomial.py --save_dir ./worlds --seed 11 --n_worlds 20 \
    --min_object 2 --max_object 6 --min_speed 0.5 --max_speed 1.5 \
    --min_std 0.05 --max_std 0.20 --min_order 3 --max_order 6
"""
import argparse
import os
import random

import numpy as np

import world_sdf
from polynomial_fit import calc_time, get_random_traj_points


def random_trajectory(args):
    """[(t, x, y), ...] for one obstacle."""
    while True:
        try:
            order = random.randint(args.min_order, args.max_order)
            x, y, points = get_random_traj_points(order=order)
        except Exception:
            continue
        break

    avg_speed = round(random.uniform(args.min_speed, args.max_speed), 1)
    std = round(random.uniform(args.min_std, args.max_std), 2)
    times = calc_time(x, y, points, avg_speed=avg_speed, min_speed=args.min_speed,
                      max_speed=args.max_speed, std=std)
    return list(zip(times, x, y))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save_dir", default="./worlds")
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--n_worlds", type=int, default=20)
    ap.add_argument("--pool", type=int, default=200, help="number of trajectories to sample from")
    ap.add_argument("--min_object", type=int, default=2)
    ap.add_argument("--max_object", type=int, default=6)
    ap.add_argument("--min_speed", type=float, required=True)
    ap.add_argument("--max_speed", type=float, required=True)
    ap.add_argument("--min_std", type=float, required=True)
    ap.add_argument("--max_std", type=float, required=True)
    ap.add_argument("--min_order", type=int, required=True)
    ap.add_argument("--max_order", type=int, required=True)
    ap.add_argument("--radius", type=float, default=0.5, help="obstacle radius (original DynaBARN: 0.5)")
    ap.add_argument("--height", type=float, default=1.0)
    ap.add_argument("--interpolation", choices=["spline", "linear"], default="spline")
    args = ap.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    os.makedirs(args.save_dir, exist_ok=True)

    pool = [random_trajectory(args) for _ in range(args.pool)]

    for i in range(args.n_worlds):
        n_obs = random.randint(args.min_object, args.max_object)
        picks = random.sample(range(len(pool)), min(n_obs, len(pool)))
        models = [world_sdf.obstacle(f"obs_{k}", pool[k], radius=args.radius,
                                     height=args.height, interpolation=args.interpolation)
                  for k in picks]
        name = f"world_{i}"
        out = os.path.join(args.save_dir, f"{name}.sdf")
        with open(out, "w") as f:
            f.write(world_sdf.world(models))
        print(f"Wrote {out} ({len(models)} obstacles)")


if __name__ == "__main__":
    main()
