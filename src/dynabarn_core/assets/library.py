"""Asset library: robots, obstacles and environments described by ``asset.yaml``.

Each asset is a folder containing an ``asset.yaml`` manifest plus optional
backend-specific files (MJCF, SDF, USD, meshes). The manifest holds the
simulator-independent description (geometry, drive, sensors) and a
``representations`` map from backend name to the file that backend loads,
with ``bindings`` mapping our generic names (``left_actuator``,
``lidar_site``, ...) to the names used inside that file.

Assets can come from three places. For a given id the first match wins,
so users override a built-in asset by reusing its id:
  1. assets added in code: ``AssetLibrary.add(ObstacleAsset(...))`` or
     ``AssetLibrary.add_manifest({...})``
  2. folders: ``AssetLibrary(search_paths=[...])`` / ``add_folder()``, then
     ``DYNABARN_ASSET_PATH`` (os.pathsep-separated)
  3. the defaults shipped inside the package (``dynabarn_core/data/assets``)
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar, Iterable, TypeVar

import yaml

from ..core.registry import Registry
from ..core.geometry import Pose2D
from ..core.shapes import Box, ExtrudedPolygon, Shape, shape_from_dict, shape_to_dict

DEFAULT_ASSET_DIR = Path(__file__).resolve().parents[1] / "data" / "assets"
ENV_VAR = "DYNABARN_ASSET_PATH"
MANIFEST = "asset.yaml"


class AssetError(LookupError):
    pass


@dataclass(frozen=True)
class Representation:
    backend: str
    path: Path
    format: str | None = None
    bindings: dict[str, str] = field(default_factory=dict)

    def binding(self, key: str, default: str | None = None) -> str:
        if key in self.bindings:
            return self.bindings[key]
        if default is not None:
            return default
        raise AssetError(f"{self.path}: missing binding '{key}' for backend '{self.backend}'")


@dataclass(frozen=True, kw_only=True)
class Asset:
    registry_name: ClassVar[str] = "asset"  # set by @ASSET_KINDS.register
    id: str
    description: str = ""
    # backend name -> simulator-specific file; empty means "build from primitives"
    representations: dict[str, Representation] = field(default_factory=dict)
    root: Path = field(default_factory=Path.cwd)  # folder relative paths resolve against

    @property
    def kind(self) -> str:
        return type(self).registry_name

    def representation(self, backend: str) -> Representation | None:
        return self.representations.get(backend)

    def require(self, backend: str) -> Representation:
        rep = self.representation(backend)
        if rep is None:
            have = ", ".join(sorted(self.representations)) or "none"
            raise AssetError(
                f"{self.kind} asset '{self.id}' has no '{backend}' representation "
                f"(available: {have}). Add one under {self.root}/asset.yaml -> representations."
            )
        return rep

    @classmethod
    def _base_kwargs(cls, data: dict, root: Path) -> dict:
        reps = {}
        for backend, r in (data.get("representations") or {}).items():
            reps[backend] = Representation(
                backend=backend,
                path=(root / r["path"]).resolve(),
                format=r.get("format"),
                bindings=dict(r.get("bindings") or {}),
            )
        return {
            "id": str(data["id"]),
            "root": root,
            "description": str(data.get("description", "")),
            "representations": reps,
        }

    @classmethod
    def from_manifest(cls, data: dict, root: Path) -> "Asset":
        return cls(**cls._base_kwargs(data, root))


ASSET_KINDS: Registry[Asset] = Registry("asset kind", "dynabarn_core.asset_kinds")


def _rgba(v: Iterable[float] | None, default=(0.8, 0.1, 0.1, 1.0)) -> tuple[float, float, float, float]:
    if v is None:
        return default
    vals = [float(x) for x in v]
    if len(vals) == 3:
        vals.append(1.0)
    return tuple(vals)  # type: ignore[return-value]


@ASSET_KINDS.register("obstacle")
@dataclass(frozen=True, kw_only=True)
class ObstacleAsset(Asset):
    geometry: Shape
    color: tuple[float, float, float, float] = (0.8, 0.1, 0.1, 1.0)
    mass: float = 1.0

    @classmethod
    def from_manifest(cls, data: dict, root: Path) -> "ObstacleAsset":
        return cls(
            **cls._base_kwargs(data, root),
            geometry=shape_from_dict(data["geometry"]),
            color=_rgba(data.get("color")),
            mass=float(data.get("mass", 1.0)),
        )


@dataclass(frozen=True)
class DiffDriveSpec:
    wheel_radius: float
    wheel_separation: float
    max_linear: float
    max_angular: float


# ---- sensors -----------------------------------------------------------
# Every sensor has a unique ``name`` (used by Simulation.subscribe/latest),
# ``enabled`` and ``rate_hz``. Realistic extras (noise, wheel odometry,
# cameras) are off by default and switched on in asset.yaml.


@dataclass(frozen=True, kw_only=True)
class SensorSpec:
    kind: ClassVar[str] = "sensor"
    name: str
    frame: str
    rate_hz: float
    enabled: bool = True

    def __post_init__(self) -> None:
        if self.rate_hz <= 0:
            raise ValueError(f"sensor '{self.name}': rate_hz must be > 0")


@dataclass(frozen=True, kw_only=True)
class LidarSpec(SensorSpec):
    kind: ClassVar[str] = "lidar"
    name: str = "lidar"
    samples: int
    angle_min: float
    angle_max: float
    range_min: float
    range_max: float
    noise_std: float = 0.0  # metres, gaussian, added to each valid range


@dataclass(frozen=True, kw_only=True)
class CameraSpec(SensorSpec):
    kind: ClassVar[str] = "camera"
    width: int
    height: int
    fov_deg: float
    depth: bool = False
    enabled: bool = False   # rendering is costly and needs OpenGL: opt in
    noise_std: float = 0.0  # rgb: intensity units (0-255); depth: metres


@dataclass(frozen=True, kw_only=True)
class ImuSpec(SensorSpec):
    kind: ClassVar[str] = "imu"
    name: str = "imu"
    frame: str = "imu_link"
    rate_hz: float = 200.0
    gyro_noise_std: float = 0.0   # rad/s   (TurtleBot3 Gazebo model: 2e-4)
    accel_noise_std: float = 0.0  # m/s^2   (TurtleBot3 Gazebo model: 1.7e-2)


@dataclass(frozen=True, kw_only=True)
class OdometrySpec(SensorSpec):
    kind: ClassVar[str] = "odometry"
    name: str = "odom"
    frame: str = "odom"
    rate_hz: float = 30.0
    # ground_truth: exact pose (no drift). wheels: integrated from wheel
    # rotation, so it drifts when wheels slip (e.g. skid-steer turns).
    source: str = "ground_truth"
    # relative (fractional) gaussian error on each step's travelled distance / rotation
    linear_noise_std: float = 0.0
    angular_noise_std: float = 0.0

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.source not in ("ground_truth", "wheels"):
            raise ValueError("odometry source must be 'ground_truth' or 'wheels'")


@dataclass(frozen=True, kw_only=True)
class JointStateSpec(SensorSpec):
    kind: ClassVar[str] = "joint_states"
    name: str = "joint_states"
    frame: str = ""
    rate_hz: float = 30.0


@dataclass(frozen=True, kw_only=True)
class ContactSpec(SensorSpec):
    kind: ClassVar[str] = "contact"
    name: str = "contact"
    frame: str = ""
    rate_hz: float = 50.0


@dataclass(frozen=True, kw_only=True)
class GroundTruthSpec(SensorSpec):
    """Exact world pose and velocity, for evaluation and debugging."""

    kind: ClassVar[str] = "ground_truth"
    name: str = "ground_truth"
    frame: str = "world"
    rate_hz: float = 50.0


def _sensor(cls, data: dict | None, where: str, **defaults):
    if data is None:
        return cls(**defaults) if defaults else None
    try:
        return cls(**{**defaults, **data})
    except TypeError as e:
        raise ValueError(f"sensors.{where}: {e}") from None


@ASSET_KINDS.register("robot")
@dataclass(frozen=True, kw_only=True)
class RobotAsset(Asset):
    footprint_radius: float
    drive: DiffDriveSpec
    lidar: LidarSpec | None = None
    cameras: tuple[CameraSpec, ...] = ()
    imu: ImuSpec | None = field(default_factory=ImuSpec)
    odometry: OdometrySpec = field(default_factory=OdometrySpec)
    joint_states: JointStateSpec = field(default_factory=JointStateSpec)
    contact: ContactSpec = field(default_factory=ContactSpec)
    ground_truth: GroundTruthSpec = field(default_factory=GroundTruthSpec)
    base_frame: str = "base_link"

    def __post_init__(self) -> None:
        names = [s.name for s in self.sensors()]
        dupes = {n for n in names if names.count(n) > 1}
        if dupes:
            raise ValueError(f"robot '{self.id}': duplicate sensor names {sorted(dupes)}")

    def sensors(self) -> tuple[SensorSpec, ...]:
        """All sensor specs, enabled or not."""
        out: list[SensorSpec] = [s for s in (self.lidar, self.imu) if s is not None]
        out += [*self.cameras, self.odometry, self.joint_states, self.contact, self.ground_truth]
        return tuple(out)

    @classmethod
    def from_manifest(cls, data: dict, root: Path) -> "RobotAsset":
        s = data.get("sensors") or {}
        base = str(data.get("base_frame", "base_link"))
        return cls(
            **cls._base_kwargs(data, root),
            footprint_radius=float(data["footprint_radius"]),
            drive=DiffDriveSpec(**{k: float(v) for k, v in data["drive"].items() if k != "type"}),
            lidar=_sensor(LidarSpec, s.get("lidar"), "lidar"),
            cameras=tuple(_sensor(CameraSpec, c, f"cameras[{i}]") for i, c in enumerate(s.get("cameras", []))),
            imu=None if s.get("imu") is False else _sensor(ImuSpec, s.get("imu"), "imu", name="imu"),
            odometry=_sensor(OdometrySpec, s.get("odometry"), "odometry", name="odom"),
            joint_states=_sensor(JointStateSpec, s.get("joint_states"), "joint_states", name="joint_states"),
            contact=_sensor(ContactSpec, s.get("contact"), "contact", name="contact"),
            ground_truth=_sensor(GroundTruthSpec, s.get("ground_truth"), "ground_truth", name="ground_truth"),
            base_frame=base,
        )


@dataclass(frozen=True)
class Wall:
    name: str
    x: float
    y: float
    yaw: float
    size: tuple[float, float, float]
    color: tuple[float, float, float, float] = (0.2, 0.3, 0.8, 1.0)


@dataclass(frozen=True)
class StaticObject:
    """A fixed piece of the environment (wall, pillar, shelf, building...).

    ``z`` is the shape's centre for box/cylinder/sphere and its base for an
    extruded polygon (as in SDF); the shape is rotated by ``yaw`` about z.
    """

    name: str
    shape: Shape
    x: float
    y: float
    z: float
    yaw: float = 0.0
    color: tuple[float, float, float, float] = (0.6, 0.6, 0.6, 1.0)

    def to_dict(self) -> dict:
        return {"name": self.name, "shape": shape_to_dict(self.shape),
                "pose": [self.x, self.y, self.z, self.yaw], "color": list(self.color)}

    @classmethod
    def from_dict(cls, d: dict) -> "StaticObject":
        pose = [float(v) for v in d["pose"]]
        if len(pose) != 4:
            raise ValueError(f"object '{d.get('name')}': pose must be [x, y, z, yaw]")
        return cls(str(d["name"]), shape_from_dict(d["shape"]), *pose,
                   color=_rgba(d.get("color"), (0.6, 0.6, 0.6, 1.0)))

    def footprint(self) -> tuple[float, float, float, float]:
        """Conservative x_min, x_max, y_min, y_max in the world."""
        r = self.shape.footprint_radius
        if isinstance(self.shape, ExtrudedPolygon):
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            xs = [self.x + c * px - s * py for px, py in self.shape.points]
            ys = [self.y + s * px + c * py for px, py in self.shape.points]
            return min(xs), max(xs), min(ys), max(ys)
        return self.x - r, self.x + r, self.y - r, self.y + r


@dataclass(frozen=True)
class Spawn:
    """A named start pose that comes with the environment."""

    id: str
    pose: Pose2D


@ASSET_KINDS.register("environment")
@dataclass(frozen=True, kw_only=True)
class EnvironmentAsset(Asset):
    ground_size: tuple[float, float] = (30.0, 30.0)
    ground_center: tuple[float, float] = (0.0, 0.0)
    walls: tuple[Wall, ...] = ()            # simple form, kept for convenience
    objects: tuple[StaticObject, ...] = ()  # general static geometry
    spawns: tuple[Spawn, ...] = ()
    # polygons (lists of x, y) covering the floor the robot can reach; empty = unknown
    free_space: tuple[tuple[tuple[float, float], ...], ...] = ()
    metadata: dict = field(default_factory=dict, compare=False)

    def static_objects(self) -> tuple[StaticObject, ...]:
        """All static geometry, with ``walls`` converted to boxes."""
        from_walls = tuple(
            StaticObject(w.name, Box(w.size), w.x, w.y, w.size[2] / 2.0, w.yaw, w.color) for w in self.walls
        )
        return from_walls + self.objects

    def spawn(self, spawn_id: str) -> Pose2D:
        for s in self.spawns:
            if s.id == spawn_id:
                return s.pose
        have = ", ".join(s.id for s in self.spawns) or "none"
        raise AssetError(f"environment '{self.id}' has no spawn '{spawn_id}' (spawns: {have})")

    def extent(self) -> tuple[float, float, float, float]:
        """x_min, x_max, y_min, y_max covering all static objects (or the ground)."""
        objs = self.static_objects()
        if not objs:
            (cx, cy), (gx, gy) = self.ground_center, self.ground_size
            return cx - gx / 2, cx + gx / 2, cy - gy / 2, cy + gy / 2
        fps = [o.footprint() for o in objs]
        return (min(f[0] for f in fps), max(f[1] for f in fps),
                min(f[2] for f in fps), max(f[3] for f in fps))

    def to_manifest(self) -> dict:
        data: dict = {"id": self.id, "kind": "environment", "description": self.description,
                      "ground": {"size": list(self.ground_size), "center": list(self.ground_center)}}
        if self.walls:
            data["walls"] = [{"name": w.name, "pose": [w.x, w.y, w.yaw], "size": list(w.size),
                              "color": list(w.color)} for w in self.walls]
        if self.spawns:
            data["spawns"] = [{"id": s.id, "x": s.pose.x, "y": s.pose.y, "yaw": s.pose.yaw} for s in self.spawns]
        if self.free_space:
            data["free_space"] = [[list(p) for p in poly] for poly in self.free_space]
        if self.metadata:
            data["metadata"] = self.metadata
        data["objects"] = [o.to_dict() for o in self.objects]
        return data

    @classmethod
    def from_manifest(cls, data: dict, root: Path) -> "EnvironmentAsset":
        walls = tuple(
            Wall(
                name=str(w["name"]),
                x=float(w["pose"][0]),
                y=float(w["pose"][1]),
                yaw=float(w["pose"][2]) if len(w["pose"]) > 2 else 0.0,
                size=tuple(float(s) for s in w["size"]),  # type: ignore[arg-type]
                color=_rgba(w.get("color"), (0.2, 0.3, 0.8, 1.0)),
            )
            for w in data.get("walls", [])
        )
        ground = data.get("ground", {}) or {}
        gx, gy = ground.get("size", [30.0, 30.0])
        cx, cy = ground.get("center", [0.0, 0.0])
        return cls(
            **cls._base_kwargs(data, root),
            ground_size=(float(gx), float(gy)),
            ground_center=(float(cx), float(cy)),
            walls=walls,
            objects=tuple(StaticObject.from_dict(o) for o in data.get("objects", []) or []),
            spawns=tuple(Spawn(str(s["id"]), Pose2D(float(s["x"]), float(s["y"]), float(s.get("yaw", 0.0))))
                         for s in data.get("spawns", []) or []),
            free_space=tuple(tuple((float(x), float(y)) for x, y in poly) for poly in data.get("free_space", []) or []),
            metadata=dict(data.get("metadata") or {}),
        )


A = TypeVar("A", bound=Asset)


class AssetLibrary:
    def __init__(
        self,
        search_paths: Iterable[str | Path] = (),
        include_env: bool = True,
        include_default: bool = True,
    ) -> None:
        paths = [Path(p) for p in search_paths]
        if include_env:
            paths += [Path(p) for p in os.environ.get(ENV_VAR, "").split(os.pathsep) if p]
        if include_default:
            paths.append(DEFAULT_ASSET_DIR)
        missing = [str(p) for p in paths[: len(paths) - include_default] if not p.is_dir()]
        if missing:
            raise AssetError(f"asset folder(s) not found: {', '.join(missing)}")
        self.search_paths: tuple[Path, ...] = tuple(p.resolve() for p in paths if p.is_dir())
        self._index: dict[str, Path] | None = None
        self._cache: dict[str, Asset] = {}
        self._added: dict[str, Asset] = {}

    # -- adding assets ---------------------------------------------------
    def add(self, *assets: Asset) -> "AssetLibrary":
        """Register assets built in code. They take priority over all folders."""
        for a in assets:
            if not isinstance(a, Asset):
                raise TypeError(f"expected an Asset, got {type(a).__name__}")
            self._added[a.id] = a
        return self

    def add_manifest(self, data: dict, root: str | Path | None = None) -> Asset:
        """Register an asset from a dict with the same layout as asset.yaml.
        Relative file paths in ``representations`` resolve against ``root``."""
        cls = ASSET_KINDS.get(data.get("kind", ""))
        asset = cls.from_manifest(data, Path(root) if root else Path.cwd())
        self.add(asset)
        return asset

    def add_folder(self, path: str | Path) -> "AssetLibrary":
        """Search ``path`` before the folders already configured."""
        p = Path(path)
        if not p.is_dir():
            raise AssetError(f"asset folder not found: {p}")
        self.search_paths = (p.resolve(), *self.search_paths)
        self._index = None
        self._cache.clear()
        return self

    def _scan(self) -> dict[str, Path]:
        if self._index is None:
            index: dict[str, Path] = {}
            for base in self.search_paths:
                for manifest in sorted(base.rglob(MANIFEST)):
                    data = yaml.safe_load(manifest.read_text()) or {}
                    asset_id = data.get("id")
                    if asset_id and asset_id not in index:
                        index[asset_id] = manifest
            self._index = index
        return self._index

    def ids(self, kind: str | None = None) -> list[str]:
        all_ids = set(self._scan()) | set(self._added)
        return sorted(i for i in all_ids if kind is None or self.get(i).kind == kind)

    def source(self, asset_id: str) -> Path | None:
        """Folder an asset was loaded from (None for assets added in code)."""
        if asset_id in self._added:
            return None
        manifest = self._scan().get(asset_id)
        if manifest is None:
            raise AssetError(f"asset '{asset_id}' not found")
        return manifest.parent

    def get(self, asset_id: str, kind: type[A] | None = None) -> A:
        if asset_id in self._added:
            self._cache[asset_id] = self._added[asset_id]
        elif asset_id not in self._cache:
            manifest = self._scan().get(asset_id)
            if manifest is None:
                raise AssetError(
                    f"asset '{asset_id}' not found. Searched: "
                    + (", ".join(map(str, self.search_paths)) or "(no asset folders)")
                )
            data = yaml.safe_load(manifest.read_text())
            cls = ASSET_KINDS.get(data.get("kind", ""))
            self._cache[asset_id] = cls.from_manifest(data, manifest.parent)
        asset = self._cache[asset_id]
        if kind is not None and not isinstance(asset, kind):
            raise AssetError(f"asset '{asset_id}' is a {asset.kind}, expected {kind.registry_name}")
        return asset  # type: ignore[return-value]

    def robot(self, asset_id: str) -> RobotAsset:
        return self.get(asset_id, RobotAsset)

    def obstacle(self, asset_id: str) -> ObstacleAsset:
        return self.get(asset_id, ObstacleAsset)

    def environment(self, asset_id: str) -> EnvironmentAsset:
        return self.get(asset_id, EnvironmentAsset)
