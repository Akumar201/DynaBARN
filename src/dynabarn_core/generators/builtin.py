"""Built-in generators (lane and random motion, as in the original DynaBARN world generators)."""
from __future__ import annotations

import dataclasses
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

    Useful for imported worlds. Starts at ``spawn`` if given, else at
    ``start_x``/``start_y``/``start_yaw`` if any of them is set, else at the
    environment's first spawn (if it has spawns).
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
        c = self.config
        env = self.assets.environment(c.environment)
        defaults = {f.name: f.default for f in dataclasses.fields(c)}
        start_given = any(getattr(c, k) != defaults[k] for k in ("start_x", "start_y", "start_yaw"))
        if c.spawn is None and env.spawns and not start_given:
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

    Each lane is cut into its free stretches (walls, static objects and the
    keep-clear zones around the robot's start and goal removed), and the
    lane's obstacles are shared out over those stretches. Obstacles sharing
    a stretch each get their own section, ``lane_gap`` apart, so they never
    meet. ``stagger_phase`` makes neighbouring lanes (and sections) start in
    opposite directions.
    """

    Config = LanesConfig
    config: LanesConfig
    MIN_SECTION = 0.2  # shortest stretch an obstacle may shuttle along (m)

    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        c = self.config
        n = c.lanes
        reg = self.region()
        # "bounds" gives the range of obstacle centres, so lanes span it edge to edge;
        # "environment" is the physical extent (walls at its edges), so lanes go in
        # the middle of n equal bands instead
        spread = _linspace if c.area == "bounds" else _band_centres
        if c.axis == "row":
            lane_pos, lo, hi = spread(reg.y_min, reg.y_max, n), reg.x_min, reg.x_max
        else:
            lane_pos, lo, hi = spread(reg.x_min, reg.x_max, n), reg.y_min, reg.y_max
        r = self.obstacle_radius
        gap = c.lane_gap + 2 * r  # centre-to-centre gap between neighbouring sections
        fs = self.free_space()
        per_lane = [0] * n
        for i in range(c.count):
            per_lane[i % n] += 1

        # sections[lane] = list of (s0, s1) centre ranges along the lane, in order
        sections: list[list[tuple[float, float]]] = []
        for lane in range(n):
            p = lane_pos[lane]
            a, b = ((lo, p), (hi, p)) if c.axis == "row" else ((p, lo), (p, hi))
            runs = [(lo + s0, lo + s1) for s0, s1 in fs.free_runs(a, b, r + c.static_clearance)]
            sections.append(self._share(runs, per_lane[lane], gap, lane, p))

        trajs: list[Trajectory] = []
        slot = [0] * n
        for i in range(c.count):
            lane, j = i % n, slot[i % n]
            slot[lane] += 1
            s0, s1 = sections[lane][j]
            p = lane_pos[lane]
            a, b = ((s0, p), (s1, p)) if c.axis == "row" else ((p, s0), (p, s1))
            if c.stagger_phase and (lane + j) % 2 == 1:
                a, b = b, a
            trajs.append(self.make_waypoints([a, b, a], rng))

        # neighbouring lanes, or the environment's own moving objects, can still be too close
        checker = self.clearance_checker()
        if checker is not None:
            for i, t in enumerate(trajs):
                d, other = checker.closest(t, r)
                if d < checker.clearance - 1e-9:
                    raise ClearanceError(
                        f"obs_{i} comes within {d:.2f} m of {other} (min_clearance={c.min_clearance}); "
                        f"use fewer lanes, a larger area, or environment_obstacles=false"
                    )
                checker.add(t, r, f"obs_{i}")
        return trajs

    def _share(self, runs: list[tuple[float, float]], k: int, gap: float, lane: int, pos: float
               ) -> list[tuple[float, float]]:
        """Split ``k`` obstacles over the free runs, widest sections first."""
        if k == 0:
            return []
        lengths = [b - a for a, b in runs]
        m = [0] * len(runs)

        def width(j: int, count: int) -> float:
            return (lengths[j] - (count - 1) * gap) / count

        for _ in range(k):
            options = [(width(j, m[j] + 1), -j) for j in range(len(runs))]
            if not options or max(options)[0] < self.MIN_SECTION:
                free = sum(max(0.0, x) for x in lengths)
                raise ClearanceError(
                    f"lane {lane} (at {pos:.2f}) has room for {sum(m)} of its {k} obstacles "
                    f"({free:.1f} m free in {len(runs)} stretch(es)); use more lanes, fewer obstacles, "
                    f"a smaller lane_gap or keep_clear"
                )
            j = -max(options)[1]
            m[j] += 1
        out = []
        for j, (a, _) in enumerate(runs):
            if m[j]:
                w = width(j, m[j])
                out += [(a + q * (w + gap), a + q * (w + gap) + w) for q in range(m[j])]
        return out


@dataclass
class RandomConfig(GeneratorConfig):
    points: int = 8
    bounce: bool = False  # random walk that reflects off its region's edges
    # cells: the area is split into a grid with one obstacle per cell; each
    #        obstacle moves randomly inside its own cell, so they never meet.
    # free:  every obstacle roams the whole area; paths are re-drawn until
    #        they keep min_clearance (only feasible for a few obstacles).
    layout: str = "cells"
    max_tries: int = 300  # layout=free: attempts per obstacle; cells: per cell (capped at 30)

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.layout not in ("cells", "free"):
            raise ValueError("layout must be 'cells' or 'free'")


@GENERATORS.register("random")
class RandomGenerator(Generator):
    """Obstacles move along random loops (waypoints or a bouncing walk).

    With ``layout=cells`` (default) each obstacle stays in its own grid cell,
    so obstacles never overlap; cells with no usable free space (walls,
    static objects, keep-clear zones) are skipped, and the grid is refined if
    too few cells are usable. With ``layout=free`` obstacles roam the whole
    area and paths are re-drawn until they keep ``min_clearance``. Every path
    is checked against static objects (``static_clearance``) and the
    environment's own moving objects.
    """

    Config = RandomConfig
    config: RandomConfig
    REFINEMENTS = 6

    def make_trajectories(self, rng: random.Random) -> list[Trajectory]:
        c = self.config
        if c.count == 0:
            return []
        if c.layout == "free":
            reg = self.region()
            regions = [(reg.x_min, reg.x_max, reg.y_min, reg.y_max)] * c.count
            out = self._place(rng, regions, c.max_tries, need=c.count)
            if out is None:
                raise ClearanceError(
                    f"layout=free could not place all {c.count} obstacles keeping min_clearance="
                    f"{c.min_clearance} m for {c.check_horizon:.0f} s after {c.max_tries} tries each. "
                    f"Use layout=cells, fewer obstacles, a shorter check_horizon or min_clearance=none "
                    f"(allows overlaps)."
                )
            return out
        cells = c.count
        for attempt in range(self.REFINEMENTS):
            regions = self._cells(rng, cells)
            out = self._place(rng, regions, min(c.max_tries, 30), need=c.count)
            if out is not None:
                return out
            cells = math.ceil(cells * 1.5)  # finer grid: more, smaller cells to choose from
        raise ClearanceError(
            f"could not place {c.count} obstacles in the free space (walls, keep_clear, "
            f"min_clearance, the environment's moving objects); use fewer obstacles, a smaller "
            f"obstacle_asset, smaller keep_clear/static_clearance, or area=environment"
        )

    def _place(self, rng, regions, tries: int, need: int) -> list[Trajectory] | None:
        """Try regions in order, one obstacle each, until ``need`` are placed."""
        c = self.config
        r = self.obstacle_radius
        checker = self.clearance_checker()
        if checker is None:
            tries = min(tries, 30)
        fs = self.free_space()
        rr = r + c.static_clearance
        trajs: list[Trajectory] = []
        for region in regions:
            if len(trajs) == need:
                break
            for _ in range(tries):
                sub = random.Random(rng.random())
                ok = lambda p: bool(fs.is_free([p], rr)[0])  # noqa: E731
                leg = lambda a, b: fs.path_free([a, b], rr)  # noqa: E731
                path = (self._bounce_path(sub, region, ok, leg) if c.bounce
                        else self._random_path(sub, region, ok, leg))
                if path is None:
                    break  # region has (almost) no free space
                if c.stagger_phase and len(trajs) % 2 == 1:
                    path = path[::-1]
                traj = self.make_waypoints(path, sub)
                if self.obstacle_free(path) and (checker is None or checker.fits(traj, r)):
                    if checker is not None:
                        checker.add(traj, r, f"obs_{len(trajs)}")
                    trajs.append(traj)
                    break
            else:
                if c.layout == "free":
                    return None
        return trajs if len(trajs) == need else None

    def _cells(self, rng: random.Random, n: int) -> list[tuple[float, float, float, float]]:
        """Grid cells (shrunk so neighbours keep their distance), spread randomly
        over the area; cells with no free space at all are dropped."""
        c = self.config
        reg = self.region()
        w, h = reg.x_max - reg.x_min, reg.y_max - reg.y_min
        nx = max(1, round(math.sqrt(n * w / h)))
        ny = math.ceil(n / nx)
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
        rects = [(reg.x_min + ix * cw + margin, reg.x_min + (ix + 1) * cw - margin,
                  reg.y_min + iy * ch + margin, reg.y_min + (iy + 1) * ch - margin) for ix, iy in cells]
        fs, rr = self.free_space(), self.obstacle_radius + c.static_clearance
        return [q for q in rects if fs.free_fraction(*q, rr, n=6) > 0]

    def _random_path(self, rng: random.Random, region, ok, leg) -> list[tuple[float, float]] | None:
        """Random waypoints in ``region``; each must be free and reachable in a
        straight free line from the previous one."""
        x0, x1, y0, y1 = region
        pts: list[tuple[float, float]] = []
        for _ in range(max(2, self.config.points)):
            for _attempt in range(50):
                p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
                if ok(p) and (not pts or leg(pts[-1], p)):
                    break
            else:
                if len(pts) >= 2:
                    break  # boxed in: keep the waypoints found so far
                return None
            pts.append(p)
        return _close(pts, leg)

    def _bounce_path(self, rng: random.Random, region, ok, leg) -> list[tuple[float, float]] | None:
        """Random walk that reflects off the region's edges and turns away
        from walls and other static objects."""
        x0, x1, y0, y1 = region
        for _attempt in range(50):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if ok((x, y)):
                break
        else:
            return None
        ang = rng.uniform(-math.pi, math.pi)
        step = max((x1 - x0 + y1 - y0) / 30.0, 0.05)
        pts = [(x, y)]
        for _ in range(max(2, self.config.points) - 1):
            for _turn in range(12):
                nx, ny, nang = x + step * math.cos(ang), y + step * math.sin(ang), ang
                bounced = False
                if nx < x0:
                    nx, nang, bounced = 2 * x0 - nx, math.pi - nang, True
                if nx > x1:
                    nx, nang, bounced = 2 * x1 - nx, math.pi - nang, True
                if ny < y0:
                    ny, nang, bounced = 2 * y0 - ny, -nang, True
                if ny > y1:
                    ny, nang, bounced = 2 * y1 - ny, -nang, True
                if leg((x, y), (nx, ny)):
                    break
                ang = rng.uniform(-math.pi, math.pi)  # blocked by a wall: try another direction
            else:
                break  # boxed in: end the walk here
            x, y, ang = nx, ny, nang
            if not bounced:
                ang += rng.uniform(-0.35, 0.35)
            pts.append((x, y))
        if len(pts) < 2:
            return None
        return _close(pts, leg)


def _close(pts: list[tuple[float, float]], leg) -> list[tuple[float, float]]:
    """Close the loop back to the start if that leg is free; otherwise retrace
    the route (out and back), which only uses legs already known to be free."""
    if leg(pts[-1], pts[0]):
        return _dedupe(pts + [pts[0]])
    return _dedupe(pts + pts[-2::-1])


def _linspace(a: float, b: float, n: int) -> list[float]:
    if n == 1:
        return [(a + b) / 2.0]
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def _band_centres(a: float, b: float, n: int) -> list[float]:
    return [a + (b - a) * (k + 0.5) / n for k in range(n)]


def _dedupe(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out = [pts[0]]
    for p in pts[1:]:
        if p != out[-1]:
            out.append(p)
    return out
