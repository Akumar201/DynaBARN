"""Where obstacles may go: a 2D free-space query built from an environment.

A point is free for a disc of radius ``r`` when it is
  * inside the region (bounds rectangle, and the environment's free-space
    outline if it has one; outlines combine even-odd, like world-collection's),
  * at least ``r`` away from every static object footprint, and
  * at least ``r`` away from every keep-clear circle (e.g. the robot's start).

All queries are vectorised with numpy over arrays of points.
"""
from __future__ import annotations

import math

import numpy as np

from ..assets import EnvironmentAsset
from ..core.geometry import Bounds
from ..core.shapes import Box, Cylinder, ExtrudedPolygon, Sphere

_CHUNK = 2_000_000  # max points x edges per batch


def _world_polygon(o) -> list[tuple[float, float]] | None:
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    if isinstance(o.shape, Box):
        hx, hy = o.shape.size[0] / 2, o.shape.size[1] / 2
        local = ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))
    elif isinstance(o.shape, ExtrudedPolygon):
        local = o.shape.points
    else:
        return None
    return [(o.x + c * x - s * y, o.y + s * x + c * y) for x, y in local]


def _edges(polys: list[list[tuple[float, float]]]) -> tuple[np.ndarray, np.ndarray]:
    """Stacked edges (E, 4) and the index of the polygon each edge belongs to."""
    segs, owner = [], []
    for k, poly in enumerate(polys):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            segs.append((x1, y1, x2, y2))
            owner.append(k)
    return np.array(segs, dtype=float).reshape(-1, 4), np.array(owner, dtype=int)


def _crossings(px: np.ndarray, py: np.ndarray, e: np.ndarray) -> np.ndarray:
    """(N, E) booleans: does a ray from each point towards +x cross each edge."""
    x1, y1, x2, y2 = e[:, 0], e[:, 1], e[:, 2], e[:, 3]
    Y = py[:, None]
    cond = (y1 > Y) != (y2 > Y)
    with np.errstate(divide="ignore", invalid="ignore"):
        xi = (x2 - x1) * (Y - y1) / (y2 - y1) + x1
    return cond & (px[:, None] < xi)


def _seg_dist(px: np.ndarray, py: np.ndarray, e: np.ndarray) -> np.ndarray:
    """(N, E) distance from each point to each segment."""
    x1, y1, x2, y2 = e[:, 0], e[:, 1], e[:, 2], e[:, 3]
    vx, vy = x2 - x1, y2 - y1
    L = vx * vx + vy * vy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(L > 0, ((px[:, None] - x1) * vx + (py[:, None] - y1) * vy) / L, 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px[:, None] - (x1 + t * vx), py[:, None] - (y1 + t * vy))


class FreeSpace:
    def __init__(
        self,
        environment: EnvironmentAsset,
        bounds: Bounds,
        keep_clear: list[tuple[float, float, float]] = (),  # (x, y, radius) circles
        max_height: float = 2.0,
    ) -> None:
        self.bounds = bounds
        polys, circles = [], []
        for o in environment.static_objects():
            bottom = o.z if isinstance(o.shape, ExtrudedPolygon) else o.z - o.shape.half_height
            if bottom > max_height:  # overhead structure: obstacles pass underneath
                continue
            poly = _world_polygon(o)
            if poly is not None:
                polys.append(poly)
            elif isinstance(o.shape, (Cylinder, Sphere)):
                circles.append((o.x, o.y, o.shape.radius))
        circles += list(keep_clear)
        self._edges, owner = _edges(polys)
        self._poly_starts = np.r_[0, np.flatnonzero(np.diff(owner)) + 1] if len(owner) else np.array([], int)
        self._circles = np.array(circles, dtype=float).reshape(-1, 3)
        self._region_edges, _ = _edges([list(p) for p in environment.free_space])

    # -- queries -------------------------------------------------------------------
    def clearance(self, pts) -> np.ndarray:
        """Distance from each point to the nearest static object or keep-clear
        circle (0 inside one)."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        out = np.full(len(pts), np.inf)
        if len(self._circles):
            c = self._circles
            d = np.hypot(pts[:, 0:1] - c[:, 0], pts[:, 1:2] - c[:, 1]) - c[:, 2]
            out = np.minimum(out, np.maximum(d, 0.0).min(axis=1))
        if len(self._edges):
            step = max(1, _CHUNK // len(self._edges))
            for i in range(0, len(pts), step):
                px, py = pts[i:i + step, 0], pts[i:i + step, 1]
                d = _seg_dist(px, py, self._edges).min(axis=1)
                cross = _crossings(px, py, self._edges)
                # crossings per polygon (edges of one polygon are contiguous): odd = inside it
                counts = np.add.reduceat(cross.astype(np.int32), self._poly_starts, axis=1)
                inside = (counts % 2 == 1).any(axis=1)
                out[i:i + step] = np.minimum(out[i:i + step], np.where(inside, 0.0, d))
        return out

    def in_region(self, pts) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        b = self.bounds
        ok = (pts[:, 0] >= b.x_min) & (pts[:, 0] <= b.x_max) & (pts[:, 1] >= b.y_min) & (pts[:, 1] <= b.y_max)
        if len(self._region_edges):
            ok &= _crossings(pts[:, 0], pts[:, 1], self._region_edges).sum(axis=1) % 2 == 1
        return ok

    def is_free(self, pts, radius: float) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        ok = self.in_region(pts)
        if ok.any():
            ok[ok] &= self.clearance(pts[ok]) > radius  # strict: inside an object (0) is never free
        return ok

    def path_free(self, points, radius: float, closed: bool = False, step: float = 0.05) -> bool:
        """Whether a disc of ``radius`` can travel along the polyline."""
        pts = [tuple(map(float, p)) for p in points]
        if closed and pts[0] != pts[-1]:
            pts.append(pts[0])
        samples = [pts[0]]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
            samples += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1)]
        return bool(self.is_free(samples, radius).all())

    def free_runs(self, a: tuple[float, float], b: tuple[float, float], radius: float,
                  step: float = 0.05) -> list[tuple[float, float]]:
        """Free stretches of the segment a->b as (start, end) distances from a."""
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(L / step)))
        s = np.linspace(0.0, L, n + 1)
        ux, uy = ((b[0] - a[0]) / L, (b[1] - a[1]) / L) if L > 0 else (0.0, 0.0)
        free = self.is_free(np.stack([a[0] + s * ux, a[1] + s * uy], axis=1), radius)
        runs, start = [], None
        for k, f in enumerate(free):
            if f and start is None:
                start = s[k]
            if (not f or k == n) and start is not None:
                end = s[k] if f else s[k - 1]
                runs.append((float(start), float(end)))
                start = None
        return runs

    def free_fraction(self, x0: float, x1: float, y0: float, y1: float, radius: float, n: int = 12) -> float:
        xs, ys = np.linspace(x0, x1, n), np.linspace(y0, y1, n)
        X, Y = np.meshgrid(xs, ys)
        return float(self.is_free(np.stack([X.ravel(), Y.ravel()], axis=1), radius).mean())
