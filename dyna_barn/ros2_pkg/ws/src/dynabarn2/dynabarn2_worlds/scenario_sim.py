"""Offline obstacle simulation for DynaBARN II worlds.

Obstacles are 2D discs that follow a behaviour (lanes, traffic, crowd) and collide with each
other, the arena bounds and static clutter. The resulting motion is saved as waypoints, so in
Gazebo obstacles visibly bounce off each other while every trial stays identical, and speeds
stay bounded because motion is generated in time rather than fitted through points.

Also provides the two checks each generated world must pass:
  earliest_arrival()   space-time search: can a robot reach the goal at all, and how soon?
  blind_collision()    chance that a robot driving straight at the goal, not reacting, is hit.
"""
import math

import numpy as np

DT = 0.01            # s, simulation step
SAVE_EVERY = 5       # keep every 5th step (0.05 s) before simplifying
ACCEL_LIMIT = 3.0    # m/s^2, how hard obstacles can steer towards their desired velocity
RESPONSE_TIME = 0.6  # s, how quickly they recover their motion after a bounce
BOUNDS = (-9.0, 9.0, -8.0, 8.0)  # x_min, x_max, y_min, y_max
ROBOT_START = (11.0, 0.0)
ROBOT_GOAL = (-9.0, 0.0)
ROBOT_RADIUS = 0.3   # m, bounding the Jackal's width; used for the checks only


class Scenario:
    """Obstacle set-up: arrays of length N, plus static discs."""

    def __init__(self, behaviour, radii, speeds, statics=(), rng=None, stop_and_go=False,
                 axis='row', lanes=7, lane_change_prob=0.0):
        self.behaviour = behaviour    # 'lanes' | 'traffic' | 'crowd'
        self.radii = np.asarray(radii, float)
        self.speeds = np.asarray(speeds, float)
        self.statics = list(statics)  # [(x, y, r), ...]
        self.rng = rng or np.random.default_rng()
        self.stop_and_go = stop_and_go
        self.axis = 0 if axis == 'row' else 1  # the axis obstacles travel along
        # An odd lane count keeps a lane on y = 0; otherwise the robot's route runs down a gap
        # between lanes and traffic along it can never get in the way.
        self.lane_centres = np.linspace(BOUNDS[2 if self.axis == 0 else 0] + 1,
                                        BOUNDS[3 if self.axis == 0 else 1] - 1, lanes)
        self.lane_change_prob = lane_change_prob  # per second, per obstacle


def _free_position(rng, radius, placed, statics, lane_value=None, axis=0):
    x_min, x_max, y_min, y_max = BOUNDS
    for _ in range(2000):
        p = np.array([rng.uniform(x_min + radius, x_max - radius),
                      rng.uniform(y_min + radius, y_max - radius)])
        if lane_value is not None:
            p[1 - axis] = lane_value
        if all(np.hypot(*(p - q)) > radius + r + 0.05 for q, r in placed) and \
                all(math.hypot(p[0] - x, p[1] - y) > radius + r + 0.05 for x, y, r in statics):
            return p
    raise RuntimeError('could not place obstacle: arena too crowded')


def simulate(sc, duration):
    """Returns times (T,) and positions (T, N, 2) of the obstacles, sampled every SAVE_EVERY steps."""
    rng, n, radii = sc.rng, len(sc.radii), sc.radii
    mass = radii ** 2
    lane_idx = rng.integers(0, len(sc.lane_centres), n)
    uses_lanes = sc.behaviour in ('lanes', 'traffic')

    placed, pos = [], np.zeros((n, 2))
    for i in range(n):
        lane = sc.lane_centres[lane_idx[i]] if uses_lanes else None
        pos[i] = _free_position(rng, radii[i], placed, sc.statics, lane, sc.axis)
        placed.append((pos[i].copy(), radii[i]))

    heading = rng.uniform(-math.pi, math.pi, n)
    direction = rng.choice([-1.0, 1.0], n)
    lane_target = sc.lane_centres[lane_idx].copy()
    vel = np.zeros((n, 2))
    moving = np.ones(n, bool)
    next_toggle = rng.uniform(1.0, 4.0, n) if sc.stop_and_go else np.full(n, np.inf)

    lo = np.array([BOUNDS[0], BOUNDS[2]]) + radii[:, None]
    hi = np.array([BOUNDS[1], BOUNDS[3]]) - radii[:, None]
    other = 1 - sc.axis
    steps = int(duration / DT)
    times, traj = [], []

    for step in range(steps):
        t = step * DT
        if step % SAVE_EVERY == 0:
            times.append(t)
            traj.append(pos.copy())

        toggles = t >= next_toggle
        if toggles.any():
            moving[toggles] = ~moving[toggles]
            next_toggle[toggles] = t + rng.uniform(1.0, 4.0, toggles.sum())

        # Desired velocity from the behaviour
        if uses_lanes:
            if sc.lane_change_prob > 0:
                for i in np.nonzero(rng.random(n) < sc.lane_change_prob * DT)[0]:
                    lane_idx[i] = np.clip(lane_idx[i] + rng.choice([-1, 1]), 0,
                                          len(sc.lane_centres) - 1)
                    lane_target[i] = sc.lane_centres[lane_idx[i]]
            # Turn around at the ends of the lane
            direction[pos[:, sc.axis] >= hi[:, sc.axis] - 0.05] = -1.0
            direction[pos[:, sc.axis] <= lo[:, sc.axis] + 0.05] = 1.0
            desired = np.zeros((n, 2))
            desired[:, sc.axis] = direction * sc.speeds
            desired[:, other] = np.clip(1.5 * (lane_target - pos[:, other]),
                                        -0.6 * sc.speeds, 0.6 * sc.speeds)
        else:
            heading += rng.normal(0.0, 0.35 * math.sqrt(DT), n)
            desired = np.stack([np.cos(heading), np.sin(heading)], 1) * sc.speeds[:, None]
        desired[~moving] = 0.0

        # Steer towards it
        accel = (desired - vel) / RESPONSE_TIME
        norm = np.linalg.norm(accel, axis=1, keepdims=True)
        accel *= np.minimum(1.0, ACCEL_LIMIT / np.maximum(norm, 1e-9))
        vel += accel * DT
        pos += vel * DT

        # Obstacle-obstacle contacts: elastic bounce, weighted by mass
        for i in range(n - 1):
            delta = pos[i + 1:] - pos[i]
            dist = np.linalg.norm(delta, axis=1)
            for k in np.nonzero(dist < radii[i + 1:] + radii[i])[0]:
                j = i + 1 + k
                normal = delta[k] / max(dist[k], 1e-9)
                overlap = radii[i] + radii[j] - dist[k]
                total = mass[i] + mass[j]
                pos[i] -= normal * overlap * mass[j] / total
                pos[j] += normal * overlap * mass[i] / total
                closing = np.dot(vel[i] - vel[j], normal)
                if closing > 0:
                    vel[i] -= 2 * mass[j] / total * closing * normal
                    vel[j] += 2 * mass[i] / total * closing * normal
                    heading[i] = math.atan2(vel[i][1], vel[i][0])
                    heading[j] = math.atan2(vel[j][1], vel[j][0])
                    # Lane traffic drives on in the direction it bounced, instead of pushing
                    # back into whatever it hit.
                    for a in (i, j):
                        if abs(vel[a][sc.axis]) > 0.05:
                            direction[a] = math.copysign(1.0, vel[a][sc.axis])

        # Static clutter
        for x, y, r in sc.statics:
            delta = pos - (x, y)
            dist = np.linalg.norm(delta, axis=1)
            for i in np.nonzero(dist < radii + r)[0]:
                normal = delta[i] / max(dist[i], 1e-9)
                pos[i] += normal * (radii[i] + r - dist[i])
                vn = np.dot(vel[i], normal)
                if vn < 0:
                    vel[i] -= 2 * vn * normal
                    heading[i] = math.atan2(vel[i][1], vel[i][0])
                    if abs(vel[i][sc.axis]) > 0.05:
                        direction[i] = math.copysign(1.0, vel[i][sc.axis])

        # Arena bounds
        for axis in (0, 1):
            below, above = pos[:, axis] < lo[:, axis], pos[:, axis] > hi[:, axis]
            hit = below | above
            if hit.any():
                pos[below, axis] = lo[below, axis]
                pos[above, axis] = hi[above, axis]
                vel[hit, axis] *= -1.0
                heading[hit] = np.arctan2(vel[hit, 1], vel[hit, 0])

    return np.array(times), np.array(traj)


def simplify(times, path, tolerance=0.01):
    """Douglas-Peucker on one obstacle's motion: indices of the samples to keep as waypoints."""
    keep = np.zeros(len(times), bool)
    keep[[0, -1]] = True
    stack = [(0, len(times) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        u = (times[a + 1:b] - times[a]) / (times[b] - times[a])
        line = path[a] + (path[b] - path[a]) * u[:, None]
        error = np.linalg.norm(path[a + 1:b] - line, axis=1)
        worst = int(np.argmax(error))
        if error[worst] > tolerance:
            mid = a + 1 + worst
            keep[mid] = True
            stack += [(a, mid), (mid, b)]
    return np.nonzero(keep)[0]


def _occupied(times, traj, radii, statics, xs, ys, t):
    """Grid of cells a robot centre cannot be in at time t."""
    k = min(np.searchsorted(times, t), len(times) - 1)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    blocked = np.zeros(gx.shape, bool)
    for p, r in list(zip(traj[k], radii)) + [((x, y), r) for x, y, r in statics]:
        blocked |= (gx - p[0]) ** 2 + (gy - p[1]) ** 2 < (r + ROBOT_RADIUS) ** 2
    return blocked


def earliest_arrival(times, traj, radii, statics=(), speed=1.0, horizon=90.0, cell=0.25):
    """Fastest possible arrival within 1 m of the goal for a robot knowing all future motion.

    Space-time flood fill on a grid: a robot moving at `speed` may step to a neighbouring cell or
    wait, each tick, if that cell is free now and next tick. Returns seconds, or None if the goal
    can't be reached within the horizon.
    """
    xs = np.arange(-9.5, 11.5 + cell, cell)
    ys = np.arange(-9.5, 9.5 + cell, cell)
    dt = cell / speed
    reach = np.zeros((len(xs), len(ys)), bool)
    reach[np.argmin(np.abs(xs - ROBOT_START[0])), np.argmin(np.abs(ys - ROBOT_START[1]))] = True
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    goal = (gx - ROBOT_GOAL[0]) ** 2 + (gy - ROBOT_GOAL[1]) ** 2 <= 1.0
    for tick in range(1, int(horizon / dt)):
        grown = reach.copy()
        grown[1:] |= reach[:-1]
        grown[:-1] |= reach[1:]
        grown[:, 1:] |= reach[:, :-1]
        grown[:, :-1] |= reach[:, 1:]
        reach = grown & ~_occupied(times, traj, radii, statics, xs, ys, tick * dt)
        if not reach.any():
            return None
        if (reach & goal).any():
            return tick * dt
    return None


def blind_collision(times, traj, radii, statics=(), speed=1.0, departures=40, horizon=60.0):
    """Fraction of departure times for which a straight-line robot, not reacting, gets hit."""
    route = math.dist(ROBOT_START, ROBOT_GOAL)
    hits = 0
    for start in np.linspace(0.0, horizon, departures, endpoint=False):
        t = np.arange(0.0, route / speed, 0.1)
        x = ROBOT_START[0] + (ROBOT_GOAL[0] - ROBOT_START[0]) * t * speed / route
        y = ROBOT_START[1] + (ROBOT_GOAL[1] - ROBOT_START[1]) * t * speed / route
        k = np.minimum(np.searchsorted(times, t + start), len(times) - 1)
        d = np.hypot(traj[k, :, 0] - x[:, None], traj[k, :, 1] - y[:, None])
        hit = (d < radii + ROBOT_RADIUS).any()
        for sx, sy, r in statics:
            hit = hit or (np.hypot(x - sx, y - sy) < r + ROBOT_RADIUS).any()
        hits += bool(hit)
    return hits / departures
