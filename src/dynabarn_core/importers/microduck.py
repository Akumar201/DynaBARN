"""Build a Microduck obstacle asset from a local clone of
pollen-robotics/microduck_rl.

    https://github.com/pollen-robotics/microduck_rl

The duck is frozen in a standing pose (a keyframe of ``scene_walk.xml``)
into one rigid MuJoCo model: every visual mesh is placed exactly where it
sits in that pose. It then moves like any other obstacle (lanes, random,
paths), facing its direction of travel. Collisions and placement use its
bounding box; the lidar sees the real duck shape.

Licences: the MJCF is Apache-2.0, but the duck's STL meshes are
CC BY-NC-SA 4.0 (non-commercial, share-alike). They are therefore never
bundled with dynabarn-core: this builder copies them from YOUR clone into
YOUR asset folder, together with their licence and an attribution notice
that says how they were modified.
"""
from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from .base import WorldImportError

REPO_URL = "https://github.com/pollen-robotics/microduck_rl"
ROBOT_DIR = Path("src/mjlab_microduck/robot/microduck")


def _quat_to_mat(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).reshape(9))
    return tuple(float(v) for v in q)  # type: ignore[return-value]


def _floats(s: str | None, default) -> list[float]:
    return [float(v) for v in s.split()] if s else list(default)


def find_robot_dir(path: Path) -> Path:
    """Accept the clone's root or its robot/microduck folder."""
    for cand in (path, path / ROBOT_DIR):
        if (cand / "robot_walk.xml").is_file() and (cand / "scene_walk.xml").is_file():
            return cand
    raise WorldImportError(f"{path}: not a microduck_rl clone (no {ROBOT_DIR}/robot_walk.xml)")


def build_microduck(source: str | Path, out_dir: str | Path, scale: float = 1.0, pose: str = "STAND",
                    asset_id: str = "microduck") -> Path:
    """Write ``<out_dir>/<asset_id>/`` (asset.yaml, mujoco/duck.xml, meshes,
    licence) and return that folder."""
    import mujoco

    if scale <= 0:
        raise ValueError("scale must be > 0")
    src = find_robot_dir(Path(source))
    model = mujoco.MjModel.from_xml_path(str(src / "scene_walk.xml"))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, pose)
    if key < 0:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, k) for k in range(model.nkey)]
        raise WorldImportError(f"no pose '{pose}' in scene_walk.xml (poses: {', '.join(n for n in names if n)})")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    root = ET.parse(src / "robot_walk.xml").getroot()
    meshdir = src / ((root.find("compiler").get("meshdir") if root.find("compiler") is not None else "") or "")
    meshes = {}
    for el in root.iter("mesh"):
        f = el.get("file")
        if f:
            meshes[el.get("name") or Path(f).stem] = (meshdir / f, _floats(el.get("scale"), (1, 1, 1)))
    materials = {el.get("name"): el.get("rgba", "0.7 0.7 0.7 1") for el in root.iter("material")}

    # each visual mesh's pose in the standing posture = its body's pose o its local pose in the XML
    pieces = []  # (mesh, material, world pos, world R)
    for body in root.iter("body"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body.get("name", ""))
        if bid < 0:
            continue
        pb, Rb = data.xpos[bid].copy(), data.xmat[bid].reshape(3, 3).copy()
        for g in body.findall("geom"):
            if g.get("class") != "visual" or not g.get("mesh"):
                continue
            if g.get("euler") or g.get("axisangle") or g.get("xyaxes") or g.get("zaxis"):
                raise WorldImportError(f"geom with unsupported orientation attribute in {body.get('name')}")
            pos = np.array(_floats(g.get("pos"), (0, 0, 0)))
            R = _quat_to_mat(_floats(g.get("quat"), (1, 0, 0, 0)))
            pieces.append((g.get("mesh"), g.get("material"), pb + Rb @ pos, Rb @ R))
    if not pieces:
        raise WorldImportError("no visual meshes found in robot_walk.xml")

    # bounding box of the standing duck (from the compiled meshes), feet on the ground
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root.find(".//worldbody/body").get("name"))
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for gi in range(model.ngeom):
        if (model.body_rootid[model.geom_bodyid[gi]] != root_id or model.geom_group[gi] != 2
                or model.geom_type[gi] != mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mid = model.geom_dataid[gi]
        v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
        v = v @ data.geom_xmat[gi].reshape(3, 3).T + data.geom_xpos[gi]
        lo, hi = np.minimum(lo, v.min(0)), np.maximum(hi, v.max(0))
    # obstacle frame: centre of the box that spans from the ground (z=0) to the top of the head
    centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2])
    ground = lo[2]
    size = (hi - lo) * scale
    height = (hi[2] - ground) * scale

    folder = Path(out_dir) / asset_id
    (folder / "mujoco" / "meshes").mkdir(parents=True, exist_ok=True)
    frag = ET.Element("mujoco", model=f"{asset_id}_obstacle")
    ET.SubElement(frag, "compiler", meshdir="meshes")
    asset = ET.SubElement(frag, "asset")
    world = ET.SubElement(frag, "worldbody")
    used_meshes, used_mats = set(), set()
    for mesh, mat, p, R in pieces:
        if mesh not in meshes:
            raise WorldImportError(f"mesh '{mesh}' is not defined in robot_walk.xml")
        rel = (p - np.array([centre[0], centre[1], ground])) * scale - np.array([0, 0, height / 2])
        attrs = {"type": "mesh", "mesh": mesh, "pos": " ".join(f"{v:.6g}" for v in rel),
                 "quat": " ".join(f"{v:.6g}" for v in _mat_to_quat(R)),
                 "contype": "0", "conaffinity": "0", "group": "2"}
        if mat:
            attrs["material"] = mat
            used_mats.add(mat)
        ET.SubElement(world, "geom", attrs)
        used_meshes.add(mesh)
    for name in sorted(used_meshes):
        path, mscale = meshes[name]
        shutil.copyfile(path, folder / "mujoco" / "meshes" / path.name)
        ET.SubElement(asset, "mesh", name=name, file=path.name,
                      scale=" ".join(f"{s * scale:.6g}" for s in mscale))
    for name in sorted(used_mats):
        ET.SubElement(asset, "material", name=name, rgba=materials.get(name, "0.7 0.7 0.7 1"))
    ET.indent(frag)
    (folder / "mujoco" / "duck.xml").write_text(
        "<!-- Microduck frozen in its " + pose + " pose, scaled x" + f"{scale:g}" + ", built by dynabarn-core from\n"
        f"     {REPO_URL}. Meshes: CC BY-NC-SA 4.0, Pollen Robotics (see NOTICE.md). -->\n"
        + ET.tostring(frag, encoding="unicode") + "\n")

    lic = src.parents[3] / "LICENSE-MODELS.md" if len(src.parents) > 3 else None
    if lic is not None and lic.is_file():
        shutil.copyfile(lic, folder / "LICENSE-MODELS.md")
    (folder / "NOTICE.md").write_text(
        f"# Microduck obstacle\n\n"
        f"The 3D meshes in `mujoco/meshes/` are by **Pollen Robotics**, from {REPO_URL},\n"
        f"licensed under **CC BY-NC-SA 4.0** (https://creativecommons.org/licenses/by-nc-sa/4.0/):\n"
        f"non-commercial use only, and adaptations must use the same licence.\n\n"
        f"Modifications: the robot was frozen in its `{pose}` pose into a single rigid model"
        f" (`mujoco/duck.xml`) and scaled by {scale:g}. The mesh files themselves are unchanged copies.\n"
        f"This folder (the meshes and the model built from them) is therefore under CC BY-NC-SA 4.0.\n"
    )
    manifest = {
        "id": asset_id,
        "kind": "obstacle",
        "description": f"Microduck (Pollen Robotics) standing, {size[0]:.2f} x {size[1]:.2f} x {height:.2f} m; "
                       f"meshes CC BY-NC-SA 4.0, see NOTICE.md",
        "geometry": {"type": "box", "size": [round(float(size[0]), 4), round(float(size[1]), 4), round(float(height), 4)]},
        "color": [0.95, 0.85, 0.2, 1.0],
        "mass": round(0.8 * scale ** 3, 4),
        "license": "CC-BY-NC-SA-4.0 (meshes), Apache-2.0 (MJCF)",
        "source": REPO_URL,
        "representations": {"mujoco": {"format": "mjcf_fragment", "path": "mujoco/duck.xml"}},
    }
    (folder / "asset.yaml").write_text(
        "# Built by `dynabarn assets import-microduck`. Collisions and placement use `geometry`\n"
        "# (the standing duck's bounding box); MuJoCo draws the real duck from mujoco/duck.xml.\n"
        + yaml.safe_dump(manifest, sort_keys=False))
    return folder
