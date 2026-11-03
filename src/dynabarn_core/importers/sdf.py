"""Import SDF worlds built from primitive shapes (box, cylinder, sphere,
polyline, plane).

Only static geometry is imported, from each collision element (visuals give
the colour). Things that cannot be represented are reported as warnings
instead of being dropped silently: non-static models, ``<include>``,
meshes, and shapes tilted in roll or pitch.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from ..assets import EnvironmentAsset, StaticObject
from ..core.shapes import Box, Cylinder, ExtrudedPolygon, Sphere
from .base import WORLD_IMPORTERS, ImportResult, WorldImporter, WorldImportError

_DEFAULT_COLOR = (0.6, 0.6, 0.6, 1.0)


@dataclass(frozen=True)
class _Pose:
    """Planar pose with height; roll/pitch must be zero."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def compose(self, child: "_Pose") -> "_Pose":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return _Pose(self.x + c * child.x - s * child.y, self.y + s * child.x + c * child.y,
                     self.z + child.z, self.yaw + child.yaw)


def _floats(text: str | None) -> list[float]:
    return [float(v) for v in (text or "").split()]


def _pose(el: ET.Element, where: str, warnings: list[str]) -> _Pose | None:
    p = el.find("pose")
    if p is None:
        return _Pose()
    if p.get("relative_to"):
        warnings.append(f"{where}: pose relative_to='{p.get('relative_to')}' is not supported; skipped")
        return None
    v = _floats(p.text)
    if len(v) == 3:
        v += [0.0, 0.0, 0.0]
    if len(v) != 6:
        warnings.append(f"{where}: pose '{p.text}' is not 'x y z roll pitch yaw'; skipped")
        return None
    if str(p.get("degrees", "")).lower() in ("true", "1"):
        v[3:] = [math.radians(a) for a in v[3:]]
    if abs(v[3]) > 1e-9 or abs(v[4]) > 1e-9:
        warnings.append(f"{where}: tilted (roll/pitch) shapes are not supported; skipped")
        return None
    return _Pose(v[0], v[1], v[2], v[5])


def _color(visual: ET.Element | None) -> tuple[float, float, float, float]:
    if visual is None:
        return _DEFAULT_COLOR
    for tag in ("material/diffuse", "material/ambient"):
        v = _floats(visual.findtext(tag))
        if len(v) >= 3:
            return tuple((v + [1.0])[:4])  # type: ignore[return-value]
    return _DEFAULT_COLOR


def _is_static(model: ET.Element) -> bool:
    return (model.findtext("static") or "").strip().lower() in ("1", "true")


def _safe_id(text: str) -> str:
    return re.sub(r"[^a-z0-9_]+", "_", text.lower()).strip("_")


@WORLD_IMPORTERS.register("sdf")
class SdfImporter(WorldImporter):
    """Any SDF world whose static models use primitive shapes."""

    @classmethod
    def can_import(cls, path: Path) -> bool:
        return path.is_file() and path.suffix in (".sdf", ".world")

    def load(self, path: Path, asset_id: str | None = None) -> ImportResult:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            raise WorldImportError(f"{path}: not valid XML ({e})") from None
        world = root.find("world") if root.tag == "sdf" else None
        if world is None:
            raise WorldImportError(f"{path}: no <sdf><world> element")
        warnings: list[str] = []
        objects: list[StaticObject] = []
        ground: tuple[tuple[float, float], tuple[float, float]] | None = None
        skipped_dynamic: list[str] = []

        for inc in world.findall("include"):
            warnings.append(f"<include> {inc.findtext('uri')!r} is not supported (only inline geometry); skipped")

        def visit(model: ET.Element, parent: _Pose, static: bool) -> None:
            nonlocal ground
            mname = model.get("name", "model")
            static = static or _is_static(model)
            mpose = _pose(model, f"model '{mname}'", warnings)
            if mpose is None:
                return
            mpose = parent.compose(mpose)
            if not static:
                skipped_dynamic.append(mname)
                return
            for sub in model.findall("model"):
                visit(sub, mpose, static)
            for link in model.findall("link"):
                lpose = _pose(link, f"model '{mname}' link '{link.get('name')}'", warnings)
                if lpose is None:
                    continue
                lpose = mpose.compose(lpose)
                visuals = link.findall("visual")
                cols = link.findall("collision")
                for i, col in enumerate(cols):
                    cname = col.get("name", f"{mname}_{i}")
                    where = f"model '{mname}' collision '{cname}'"
                    cpose = _pose(col, where, warnings)
                    if cpose is None:
                        continue
                    pose = lpose.compose(cpose)
                    vis = visuals[i] if len(visuals) == len(cols) else (visuals[0] if visuals else None)
                    color = _color(vis)
                    geom = col.find("geometry")
                    if geom is None or len(geom) == 0:
                        warnings.append(f"{where}: no geometry; skipped")
                        continue
                    g = geom[0]
                    is_ground = "ground" in mname.lower()
                    if g.tag == "plane" or (is_ground and g.tag == "box"):
                        size = _floats(g.findtext("size")) or [100.0, 100.0]
                        ground = ((pose.x, pose.y), (size[0], size[1]))
                        continue
                    name = cname if cname not in names else f"{mname}/{cname}"
                    names.add(name)
                    if g.tag == "box":
                        sx, sy, sz = _floats(g.findtext("size"))
                        objects.append(StaticObject(name, Box((sx, sy, sz)), pose.x, pose.y, pose.z, pose.yaw, color))
                    elif g.tag == "cylinder":
                        shape = Cylinder(float(g.findtext("radius")), float(g.findtext("length")))
                        objects.append(StaticObject(name, shape, pose.x, pose.y, pose.z, pose.yaw, color))
                    elif g.tag == "sphere":
                        shape = Sphere(float(g.findtext("radius")))
                        objects.append(StaticObject(name, shape, pose.x, pose.y, pose.z, pose.yaw, color))
                    elif g.tag == "polyline":
                        for k, poly in enumerate(geom.findall("polyline")):
                            pts = tuple(tuple(_floats(p.text)[:2]) for p in poly.findall("point"))
                            shape = ExtrudedPolygon(pts, float(poly.findtext("height") or 1.0))  # type: ignore[arg-type]
                            pname = name if k == 0 else f"{name}#{k}"
                            objects.append(StaticObject(pname, shape, pose.x, pose.y, pose.z, pose.yaw, color))
                    else:
                        warnings.append(f"{where}: <{g.tag}> geometry is not supported; skipped")

        names: set[str] = set()
        for model in world.findall("model"):
            visit(model, _Pose(), False)
        if skipped_dynamic:
            warnings.append(f"{len(skipped_dynamic)} non-static model(s) skipped (moving objects are not "
                            f"imported from SDF): {', '.join(skipped_dynamic[:5])}"
                            + (" ..." if len(skipped_dynamic) > 5 else ""))
        if not objects:
            raise WorldImportError(f"{path}: no static geometry found")

        env_id = asset_id or _safe_id(world.get("name") or path.stem)
        tmp = EnvironmentAsset(id=env_id, objects=tuple(objects))
        if ground is None:  # no ground in the file: cover the geometry with a margin
            x0, x1, y0, y1 = tmp.extent()
            ground = (((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0 + 10.0, y1 - y0 + 10.0))
        env = EnvironmentAsset(
            id=env_id,
            description=f"Imported from {path.name}",
            ground_center=ground[0],
            ground_size=ground[1],
            objects=tuple(objects),
            metadata={"source": {"importer": "sdf", "path": str(path), "world": world.get("name")},
                      "skipped_dynamic_models": skipped_dynamic},
        )
        return ImportResult(env, warnings)
