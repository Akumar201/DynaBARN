"""DynaBARN II worlds: lane, random and traffic (lane-change) obstacle motion.

Merges DynaBARN/generate_grid_plugins_gz11.py and generate_dynabarn_traffic.py
for Gazebo Harmonic. Writes one world with --count obstacles.

Movement modes:
  controlled  back-and-forth lanes             --axis row|column
  random      random closed path in bounds     [--rand_bounce]
  traffic     lanes with random lane changes   --axis row|column

Examples:
  generate_movers.py --movement controlled --axis row --rows 6 \
    --passes 3 --speed 1.2 --count 36 --stagger_phase
  generate_movers.py --movement random --rand_bounce --rand_points 30 \
    --speed_min 0.6 --speed_max 1.8 --speed_mode agent --count 40
  generate_movers.py --movement traffic --axis row --passes 4 --count 36 \
    --lane_change_prob 0.4 --traffic_samples 50 --stagger_phase
"""
import argparse
import math
import os
import random

import numpy as np

from . import world_sdf


# ---------- Path utilities ----------
def _dedupe(xs, ys):
    path = [(xs[0], ys[0])]
    for i in range(1, len(xs)):
        if (xs[i], ys[i]) != path[-1]:
            path.append((xs[i], ys[i]))
    return [p[0] for p in path], [p[1] for p in path]


def _append_point(xs, ys, x, y):
    if not xs or (xs[-1], ys[-1]) != (x, y):
        xs.append(x)
        ys.append(y)


def _extend(xs, ys, leg):
    for x, y in zip(*leg):
        _append_point(xs, ys, x, y)


def _smoothstep01(u):
    return u * u * (3 - 2 * u)


def _lane_centers(vmin, vmax, n):
    return [(vmin + vmax) / 2.0] if n <= 1 else list(np.linspace(vmin, vmax, n))


def row_path(y, x_min, x_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x_min, x_max, x_min]
        ys += [y, y, y]
    return _dedupe(xs, ys)


def col_path(x, y_min, y_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x, x, x]
        ys += [y_min, y_max, y_min]
    return _dedupe(xs, ys)


def random_path(x_min, x_max, y_min, y_max, points, bounce=False, seed=None):
    """Random closed path in bounds: random waypoints, or a random walk bouncing off the borders."""
    rng = random.Random(seed)
    points = max(2, points)

    if not bounce:
        xs = [rng.uniform(x_min, x_max) for _ in range(points)]
        ys = [rng.uniform(y_min, y_max) for _ in range(points)]
        xs.append(xs[0])  # close the loop
        ys.append(ys[0])
        return _dedupe(xs, ys)

    cx = rng.uniform(x_min, x_max)
    cy = rng.uniform(y_min, y_max)
    ang = rng.uniform(-math.pi, math.pi)
    step = max((x_max - x_min + y_max - y_min) / 30.0, 0.2)
    xs, ys = [cx], [cy]
    for _ in range(points - 1):
        cx += step * math.cos(ang)
        cy += step * math.sin(ang)
        bounced = False
        if cx < x_min:
            cx, ang, bounced = x_min + (x_min - cx), math.pi - ang, True
        if cx > x_max:
            cx, ang, bounced = x_max - (cx - x_max), math.pi - ang, True
        if cy < y_min:
            cy, ang, bounced = y_min + (y_min - cy), -ang, True
        if cy > y_max:
            cy, ang, bounced = y_max - (cy - y_max), -ang, True
        # small angle jitter so paths don't become degenerate
        if not bounced:
            ang += rng.uniform(-0.35, 0.35)
        xs.append(cx)
        ys.append(cy)
    xs.append(xs[0])
    ys.append(ys[0])
    return _dedupe(xs, ys)


def _lane_leg(lane0, lane1, along0, along1, samples, axis):
    """Leg moving along one axis while easing from lane0 to lane1 on the other."""
    xs, ys = [], []
    samples = max(1, samples)
    for i in range(samples + 1):
        u = i / samples
        along = along0 + u * (along1 - along0)
        s = _smoothstep01(u)
        lane = (1 - s) * lane0 + s * lane1
        x, y = (along, lane) if axis == "row" else (lane, along)
        _append_point(xs, ys, x, y)
    return xs, ys


def traffic_path(i, args, lanes, rng):
    """Back-and-forth lane traffic that sometimes changes to a neighbouring lane."""
    if args.axis == "row":
        lo, hi = args.x_min, args.x_max
    else:
        lo, hi = args.y_min, args.y_max
    lane = i % len(lanes)
    pos = float(lanes[lane])
    forward = (i % 2 == 0)
    xs, ys = [], []
    for _ in range(args.passes * 2):  # two legs per pass
        a0, a1 = (lo, hi) if forward else (hi, lo)
        if rng.random() < args.lane_change_prob and len(lanes) > 1:
            candidates = [k for k in (lane - 1, lane + 1) if 0 <= k < len(lanes)]
            lane = rng.choice(candidates)
            tgt = float(lanes[lane])
            aA = a0 + 0.33 * (a1 - a0)
            aC = a0 + 0.66 * (a1 - a0)
            _extend(xs, ys, _lane_leg(pos, pos, a0, aA, args.traffic_samples // 3, args.axis))
            _extend(xs, ys, _lane_leg(pos, tgt, aA, aC, args.traffic_samples // 2, args.axis))
            _extend(xs, ys, _lane_leg(tgt, tgt, aC, a1, args.traffic_samples // 3, args.axis))
            pos = tgt
        else:
            _extend(xs, ys, _lane_leg(pos, pos, a0, a1, max(8, args.traffic_samples), args.axis))
        forward = not forward
    return xs, ys


def build_times(xs, ys, speed, dwell=0.25, speed_min=None, speed_max=None, rng=None):
    """Constant or segment-wise random speed. Returns strictly increasing times."""
    if rng is None:
        rng = random.Random()
    ts = [0.0]
    for i in range(len(xs) - 1):
        d = math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
        seg_speed = speed
        if speed_min is not None and speed_max is not None and speed_max > 0:
            seg_speed = rng.uniform(speed_min, speed_max)
        ts.append(ts[-1] + d / max(1e-6, seg_speed))
    ts[-1] += dwell
    return ts


# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save_dir", default="./worlds")
    ap.add_argument("--name", default="world_movers", help="output file stem")
    # bounds & lanes
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--x_min", type=float, default=-9.0)
    ap.add_argument("--x_max", type=float, default=9.0)
    ap.add_argument("--y_min", type=float, default=-8.0)
    ap.add_argument("--y_max", type=float, default=8.0)
    # population & kinematics
    ap.add_argument("--count", type=int, default=36, help="number of moving obstacles")
    ap.add_argument("--passes", type=int, default=3, help="lane passes (controlled/traffic)")
    ap.add_argument("--speed", type=float, default=1.0, help="constant speed (m/s)")
    ap.add_argument("--speed_min", type=float, default=None)
    ap.add_argument("--speed_max", type=float, default=None)
    ap.add_argument("--speed_mode", choices=["constant", "segment", "agent"], default="constant",
                    help="constant: --speed; segment: random per segment; agent: random per obstacle")
    ap.add_argument("--dwell", type=float, default=0.25, help="pause at loop end (s)")
    ap.add_argument("--stagger_phase", action="store_true")
    ap.add_argument("--seed", type=int, default=11)
    # movement
    ap.add_argument("--movement", choices=["controlled", "random", "traffic"], default="controlled")
    ap.add_argument("--axis", choices=["row", "column"], default="row")
    ap.add_argument("--rand_points", type=int, default=8, help="waypoints for random paths (>=2)")
    ap.add_argument("--rand_bounce", action="store_true", help="bouncing random walk")
    ap.add_argument("--lane_change_prob", type=float, default=0.35)
    ap.add_argument("--traffic_samples", type=int, default=40)
    # obstacle shape
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--interpolation", choices=["spline", "linear"], default="spline")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    os.makedirs(args.save_dir, exist_ok=True)

    y_lanes = _lane_centers(args.y_min, args.y_max, max(1, args.rows))
    x_lanes = _lane_centers(args.x_min, args.x_max, max(1, args.cols))

    models = []
    for i in range(args.count):
        if args.movement == "random":
            xs, ys = random_path(args.x_min, args.x_max, args.y_min, args.y_max,
                                 points=args.rand_points, bounce=args.rand_bounce,
                                 seed=args.seed + i)
            if args.stagger_phase and i % 2 == 1:
                xs, ys = xs[::-1], ys[::-1]
        elif args.movement == "controlled":
            if args.axis == "column":
                xs, ys = col_path(float(x_lanes[i % len(x_lanes)]), args.y_min, args.y_max, args.passes)
                if args.stagger_phase and i % 2 == 1:
                    xs, ys = xs[::-1], ys[::-1]
            else:
                xs, ys = row_path(float(y_lanes[i % len(y_lanes)]), args.x_min, args.x_max, args.passes)
                if args.stagger_phase and i % 2 == 0:
                    xs, ys = xs[::-1], ys[::-1]
        else:
            xs, ys = traffic_path(i, args, y_lanes if args.axis == "row" else x_lanes, rng)

        has_range = args.speed_min is not None and args.speed_max is not None
        if args.speed_mode == "segment" and has_range:
            ts = build_times(xs, ys, args.speed, args.dwell, args.speed_min, args.speed_max, rng)
        elif args.speed_mode == "agent" and has_range:
            ts = build_times(xs, ys, rng.uniform(args.speed_min, args.speed_max), args.dwell, rng=rng)
        else:
            ts = build_times(xs, ys, args.speed, args.dwell, rng=rng)

        models.append(world_sdf.obstacle(f"obs_{i}", list(zip(ts, xs, ys)), radius=args.radius,
                                         height=args.height, interpolation=args.interpolation))

    out = os.path.join(args.save_dir, f"{args.name}.sdf")
    with open(out, "w") as f:
        f.write(world_sdf.world(models))
    print(f"Wrote {out} ({len(models)} obstacles)")


if __name__ == "__main__":
    main()
