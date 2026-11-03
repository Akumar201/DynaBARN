"""Basic geometric value types. Units: metres, radians. Frame: world, z up."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}

    @classmethod
    def from_dict(cls, d: dict) -> "Pose2D":
        return cls(float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)))

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(other.x - self.x, other.y - self.y)


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned region where obstacles move."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError(f"invalid bounds {self}")

    def contains(self, x: float, y: float, margin: float = 1e-9) -> bool:
        return (
            self.x_min - margin <= x <= self.x_max + margin
            and self.y_min - margin <= y <= self.y_max + margin
        )

    def to_dict(self) -> dict:
        return {"x_min": self.x_min, "x_max": self.x_max, "y_min": self.y_min, "y_max": self.y_max}

    @classmethod
    def from_dict(cls, d: dict) -> "Bounds":
        return cls(**{k: float(d[k]) for k in ("x_min", "x_max", "y_min", "y_max")})


def yaw_to_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


# ---- polygons --------------------------------------------------------------
Point2 = tuple[float, float]


def polygon_area(points: list[Point2] | tuple[Point2, ...]) -> float:
    """Signed area (positive when counter-clockwise)."""
    n = len(points)
    return 0.5 * sum(points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1]
                     for i in range(n))


def _cross(o: Point2, a: Point2, b: Point2) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _in_triangle(p: Point2, a: Point2, b: Point2, c: Point2) -> bool:
    return _cross(a, b, p) >= -1e-12 and _cross(b, c, p) >= -1e-12 and _cross(c, a, p) >= -1e-12


def triangulate(points: list[Point2] | tuple[Point2, ...]) -> list[tuple[Point2, Point2, Point2]]:
    """Split a simple (non-self-intersecting) polygon, convex or not, into
    counter-clockwise triangles by ear clipping. Collinear and repeated
    vertices are dropped first."""
    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    if polygon_area(pts) < 0:
        pts.reverse()
    # drop repeated and collinear vertices
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if b == a or abs(_cross(a, b, c)) < 1e-12:
                pts.pop(i)
                changed = True
                break
    if len(pts) < 3 or abs(polygon_area(pts)) < 1e-12:
        raise ValueError("polygon has no area")

    triangles = []
    idx = list(range(len(pts)))
    guard = 0
    while len(idx) > 3:
        guard += 1
        if guard > 10 * len(pts) ** 2:
            raise ValueError("polygon could not be triangulated (self-intersecting?)")
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if _cross(a, b, c) <= 1e-12:  # reflex or degenerate corner: not an ear
                continue
            if any(_in_triangle(pts[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            triangles.append((a, b, c))
            idx.pop(k)
            break
        else:
            raise ValueError("polygon could not be triangulated (self-intersecting?)")
    triangles.append(tuple(pts[i] for i in idx))  # type: ignore[arg-type]
    return triangles
