"""Primitive collision/visual geometry shared by all backends.

Every shape is defined in its own frame. For ``Box``, ``Cylinder`` and
``Sphere`` the frame origin is the shape's centre; for ``ExtrudedPolygon``
it is the base, with the polygon extruded upwards by ``height`` (as SDF's
``<polyline>``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Union

from .geometry import polygon_area, triangulate


@dataclass(frozen=True)
class Cylinder:
    radius: float
    length: float

    @property
    def half_height(self) -> float:
        return self.length / 2.0

    @property
    def footprint_radius(self) -> float:
        return self.radius


@dataclass(frozen=True)
class Box:
    size: tuple[float, float, float]  # full extents x, y, z

    @property
    def half_height(self) -> float:
        return self.size[2] / 2.0

    @property
    def footprint_radius(self) -> float:
        return 0.5 * (self.size[0] ** 2 + self.size[1] ** 2) ** 0.5


@dataclass(frozen=True)
class Sphere:
    radius: float

    @property
    def half_height(self) -> float:
        return self.radius

    @property
    def footprint_radius(self) -> float:
        return self.radius


@dataclass(frozen=True)
class ExtrudedPolygon:
    """A 2D polygon (convex or not) extruded upwards from its frame origin."""

    points: tuple[tuple[float, float], ...]
    height: float

    def __post_init__(self) -> None:
        pts = tuple((float(x), float(y)) for x, y in self.points)
        object.__setattr__(self, "points", pts)
        if len(pts) < 3 or abs(polygon_area(pts)) < 1e-12:
            raise ValueError("extruded polygon needs at least 3 points enclosing an area")
        if self.height <= 0:
            raise ValueError("extruded polygon height must be > 0")

    @property
    def half_height(self) -> float:
        return self.height / 2.0

    @property
    def footprint_radius(self) -> float:
        return max(math.hypot(x, y) for x, y in self.points)

    def convex_pieces(self) -> list[tuple[tuple[float, float], ...]]:
        """Convex 2D pieces covering the polygon, for simulators that only
        collide convex shapes (MuJoCo)."""
        return [tuple(t) for t in triangulate(self.points)]


Shape = Union[Cylinder, Box, Sphere, ExtrudedPolygon]


def shape_from_dict(d: dict) -> Shape:
    kind = d.get("type")
    if kind == "cylinder":
        return Cylinder(float(d["radius"]), float(d["length"]))
    if kind == "box":
        sx, sy, sz = (float(v) for v in d["size"])
        return Box((sx, sy, sz))
    if kind == "sphere":
        return Sphere(float(d["radius"]))
    if kind == "polygon":
        return ExtrudedPolygon(tuple(tuple(p) for p in d["points"]), float(d["height"]))
    raise ValueError(f"unknown geometry type {kind!r} (expected box, cylinder, sphere or polygon)")


def shape_to_dict(s: Shape) -> dict:
    if isinstance(s, Cylinder):
        return {"type": "cylinder", "radius": s.radius, "length": s.length}
    if isinstance(s, Box):
        return {"type": "box", "size": list(s.size)}
    if isinstance(s, Sphere):
        return {"type": "sphere", "radius": s.radius}
    if isinstance(s, ExtrudedPolygon):
        return {"type": "polygon", "points": [list(p) for p in s.points], "height": s.height}
    raise TypeError(s)
