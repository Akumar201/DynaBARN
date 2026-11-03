"""Built-in generators (lane and random motion, as in the original DynaBARN world generators)."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from ..core.geometry import Bounds, Pose2D
from ..core.trajectories import Trajectory
from .base import GENERATORS, ClearanceError, Generator, GeneratorConfig


@dataclass
class EmptyConfig(GeneratorConfig):
    count: int = 0


@GENERATORS.register("empty")
class EmptyGenerator(Generator):
    """No moving obstacles: just the environment and the robot.

    Useful for imported worlds. Starts at the environment's first spawn
    when it has spawns and no start is given.
    """

    Config = EmptyConfig
    config: EmptyConfig

    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        if self.config.count:
            raise ValueError("the empty generator has no obstacles; leave count at 0")
        return []

    def scenario_bounds(self) -> Bounds:
        x0, x1, y0, y1 = self.assets.environment(self.config.environment).extent()
        return Bounds(x0, x1, y0, y1)

    def start_pose(self) -> Pose2D:
        env = self.assets.environment(self.config.environment)
        if self.config.spawn is None and env.spawns:
            return env.spawns[0].pose
        return super().start_pose()


@dataclass
class LanesConfig(GeneratorConfig):
    axis: str = "row"  # row: obstacles move along x; column: along y
    lanes: int = 6
    # free space between the surfaces of obstacles sharing a lane (m)
    lane_gap: float = 1.0

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.axis not in ("row", "column"):
            raise ValueError("axis must be 'row' or 'column'")
        if self.lanes < 1:
            raise ValueError("lanes must be >= 1")
        if self.lane_gap < 0:
            raise ValueError("lane_gap must be >= 0")


@GENERATORS.register("lanes")
class LanesGenerator(Generator):
    """Obstacles shuttle back and forth along evenly spaced lanes.

    When several obstacles share a lane, the lane is split into equal
    sections ``lane_gap`` apart and each obstacle shuttles inside its own
    section, so obstacles in a lane never meet. ``stagger_phase`` makes
    neighbouring lanes (and sections) start in opposite directions.
    """

    Config = LanesConfig
    config: LanesConfig

    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        c = self.config
        n = c.lanes
        if c.axis == "row":
            lane_pos, lo, hi = _linspace(c.y_min, c.y_max, n), c.x_min, c.x_max
        else:
            lane_pos, lo, hi = _linspace(c.x_min, c.x_max, n), c.y_min, c.y_max
        # centre-to-centre gap between neighbouring sections
        gap = c.lane_gap + 2 * self.obstacle_radius
        per_lane = [0] * n
        for i in range(c.count):
            per_lane[i % n] += 1
        for k in per_lane:
            if k and (hi - lo - (k - 1) * gap) / k <= 0:
                raise ClearanceError(
                    f"{k} obstacles per lane with lane_gap={c.lane_gap} m do not fit in "
                    f"{hi - lo:.1f} m; use more lanes, fewer obstacles or a smaller lane_gap"
                )

        trajs: list[Trajectory] = []
        slot = [0] * n
        for i in range(c.count):
            lane, j = i % n, slot[i % n]
            slot[lane] += 1
            k = per_lane[lane]
            width = (hi - lo - (k - 1) * gap) / k
            s0 = lo + j * (width + gap)
            s1 = s0 + width
            p = lane_pos[lane]
            a, b = ((s0, p), (s1, p)) if c.axis == "row" else ((p, s0), (p, s1))
            if c.stagger_phase and (lane + j) % 2 == 1:
                a, b = b, a
            trajs.append(self.make_waypoints([a, b, a], rng))

        # Neighbouring lanes can still be too close (e.g. many lanes, big obstacles).
        checker = self.clearance_checker()
        if checker is not None:
            for i, t in enumerate(trajs):
                d, j = checker.closest(t)
                if d < checker.min_distance - 1e-9:
                    raise ClearanceError(
                        f"obs_{i} comes within {d - 2 * self.obstacle_radius:.2f} m of obs_{j}; "
                        f"lanes are too close for min_clearance={c.min_clearance}. "
                        f"Use fewer lanes or a larger area."
                    )
                checker.add(t)
        return trajs


@dataclass
class RandomConfig(GeneratorConfig):
    points: int = 8
    bounce: bool = False  # random walk that reflects off its region's edges
    # cells: the area is split into a grid with one obstacle per cell; each
    #        obstacle moves randomly inside its own cell, so they never meet.
    # free:  every obstacle roams the whole area; paths are re-drawn until
    #        they keep min_clearance (only feasible for a few obstacles).
    layout: str = "cells"
    max_tries: int = 300  # layout=free: attempts per obstacle

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.layout not in ("cells", "free"):
            raise ValueError("layout must be 'cells' or 'free'")


@GENERATORS.register("random")
class RandomGenerator(Generator):
    """Obstacles move along random loops (waypoints or a bouncing walk).

    With ``layout=cells`` (default) each obstacle stays in its own grid cell,
    so obstacles never overlap. With ``layout=free`` they roam the whole area
    and paths are re-drawn until they keep ``min_clearance``.
    """

    Config = RandomConfig
    config: RandomConfig

    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        c = self.config
        if c.layout == "cells":
            regions = self._cells(rng)
            tries = 1
        else:
            regions = [(c.x_min, c.x_max, c.y_min, c.y_max)] * c.count
            tries = c.max_tries
        checker = self.clearance_checker()
        if checker is None:
            tries = 1
        trajs: list[Trajectory] = []
        for i, region in enumerate(regions):
            best = None
            for _ in range(tries):
                sub = random.Random(rng.random())
                path = self._bounce_path(sub, region) if c.bounce else self._random_path(sub, region)
                if c.stagger_phase and i % 2 == 1:
                    path = path[::-1]
                traj = self.make_waypoints(path, sub)
                if checker is None or checker.fits(traj):
                    best = traj
                    break
            if best is None:
                if c.layout == "cells":  # cannot happen unless cells are mis-sized
                    raise AssertionError(f"cell layout produced an overlap for obs_{i}")
                raise ClearanceError(
                    f"placed only {i} of {c.count} obstacles: no path for obs_{i} keeps "
                    f"min_clearance={c.min_clearance} m for {c.check_horizon:.0f} s after "
                    f"{c.max_tries} tries. Use layout=cells, fewer obstacles, a shorter "
                    f"check_horizon or min_clearance=none (allows overlaps)."
                )
            if checker is not None:
                checker.add(best)
            trajs.append(best)
        return trajs

    def _cells(self, rng: random.Random) -> list[tuple[float, float, float, float]]:
        """One shrunken grid cell per obstacle, spread randomly over the area."""
        c = self.config
        if c.count == 0:
            return []
        w, h = c.x_max - c.x_min, c.y_max - c.y_min
        nx = max(1, round(math.sqrt(c.count * w / h)))
        ny = math.ceil(c.count / nx)
        cw, ch = w / nx, h / ny
        # keep each obstacle's surface min_clearance/2 away from its cell edge
        margin = self.obstacle_radius + (c.min_clearance or 0.0) / 2 if c.min_clearance is not None else 0.0
        if cw <= 2 * margin or ch <= 2 * margin:
            raise ClearanceError(
                f"{c.count} obstacles need {nx}x{ny} cells of {cw:.2f} x {ch:.2f} m, too small for "
                f"obstacles of radius {self.obstacle_radius} m with min_clearance={c.min_clearance}. "
                f"Use fewer obstacles, a larger area or a smaller min_clearance."
            )
        cells = [(ix, iy) for ix in range(nx) for iy in range(ny)]
        rng.shuffle(cells)
        return [
            (c.x_min + ix * cw + margin, c.x_min + (ix + 1) * cw - margin,
             c.y_min + iy * ch + margin, c.y_min + (iy + 1) * ch - margin)
            for ix, iy in cells[: c.count]
        ]

    def _random_path(self, rng: random.Random, region) -> list[tuple[float, float]]:
        x0, x1, y0, y1 = region
        pts = [(rng.uniform(x0, x1), rng.uniform(y0, y1)) for _ in range(max(2, self.config.points))]
        return _dedupe(pts + [pts[0]])

    def _bounce_path(self, rng: random.Random, region) -> list[tuple[float, float]]:
        x0, x1, y0, y1 = region
        x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
        ang = rng.uniform(-math.pi, math.pi)
        step = max((x1 - x0 + y1 - y0) / 30.0, 0.05)
        pts = [(x, y)]
        for _ in range(max(2, self.config.points) - 1):
            x += step * math.cos(ang)
            y += step * math.sin(ang)
            bounced = False
            if x < x0:
                x, ang, bounced = 2 * x0 - x, math.pi - ang, True
            if x > x1:
                x, ang, bounced = 2 * x1 - x, math.pi - ang, True
            if y < y0:
                y, ang, bounced = 2 * y0 - y, -ang, True
            if y > y1:
                y, ang, bounced = 2 * y1 - y, -ang, True
            if not bounced:
                ang += rng.uniform(-0.35, 0.35)
            pts.append((x, y))
        return _dedupe(pts + [pts[0]])


def _linspace(a: float, b: float, n: int) -> list[float]:
    if n == 1:
        return [(a + b) / 2.0]
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def _dedupe(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out = [pts[0]]
    for p in pts[1:]:
        if p != out[-1]:
            out.append(p)
    return out
