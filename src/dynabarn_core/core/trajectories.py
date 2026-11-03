"""Obstacle trajectories (Strategy pattern).

Every trajectory answers ``pose_at(t)``. Backends that cannot run Python at
simulation time (e.g. a gz-sim C++ system) use the universal export form,
``as_waypoints()``, which every trajectory provides. A backend may also
handle a type natively by checking ``trajectory.type``.

Add a new type by subclassing ``Trajectory`` and decorating it with
``@TRAJECTORIES.register("name")``.
"""
from __future__ import annotations

import bisect
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import ClassVar, Sequence

import numpy as np

from .geometry import Pose2D
from .registry import Registry


class Trajectory(ABC):
    registry_name: ClassVar[str]

    @abstractmethod
    def pose_at(self, t: float) -> Pose2D:
        """Pose at simulation time ``t`` (seconds, t >= 0)."""

    @property
    @abstractmethod
    def period(self) -> float | None:
        """Loop period in seconds, or None if the motion does not repeat."""

    @abstractmethod
    def params(self) -> dict:
        """Constructor arguments as plain data (for serialisation)."""

    @property
    def type(self) -> str:
        return self.registry_name

    def to_dict(self) -> dict:
        return {"type": self.type, **self.params()}

    def positions(self, times: "np.ndarray") -> "np.ndarray":
        """(N, 2) array of x, y at ``times``. Subclasses may vectorise this."""
        return np.array([(p.x, p.y) for p in map(self.pose_at, times)], dtype=float).reshape(-1, 2)

    def sample(self, dt: float, horizon: float | None = None) -> list[tuple[float, Pose2D]]:
        if dt <= 0:
            raise ValueError("dt must be > 0")
        end = horizon if horizon is not None else (self.period or 0.0)
        n = max(1, int(math.ceil(end / dt)))
        return [(i * dt, self.pose_at(i * dt)) for i in range(n + 1)]

    def as_waypoints(self, dt: float = 0.1) -> "WaypointTrajectory":
        """Universal export form: a looping, time-stamped polyline."""
        if self.period is None:
            raise ValueError(f"{self.type} trajectory is not periodic; pass a horizon to sample()")
        samples = self.sample(dt, self.period)
        return WaypointTrajectory(
            times=tuple(t for t, _ in samples),
            points=tuple((p.x, p.y) for _, p in samples),
            loop=True,
        )


TRAJECTORIES: Registry[Trajectory] = Registry("trajectory", "dynabarn_core.trajectories")


def trajectory_from_dict(d: dict) -> Trajectory:
    d = dict(d)
    cls = TRAJECTORIES.get(d.pop("type"))
    return cls.from_params(d)  # type: ignore[attr-defined]


@TRAJECTORIES.register("static")
@dataclass(frozen=True)
class StaticTrajectory(Trajectory):
    pose: Pose2D

    def pose_at(self, t: float) -> Pose2D:
        return self.pose

    @property
    def period(self) -> float | None:
        return 0.0

    def params(self) -> dict:
        return {"pose": self.pose.to_dict()}

    @classmethod
    def from_params(cls, p: dict) -> "StaticTrajectory":
        return cls(Pose2D.from_dict(p["pose"]))

    def as_waypoints(self, dt: float = 0.1) -> "WaypointTrajectory":
        return WaypointTrajectory(times=(0.0,), points=((self.pose.x, self.pose.y),), loop=False)


@TRAJECTORIES.register("waypoints")
@dataclass(frozen=True)
class WaypointTrajectory(Trajectory):
    """Piecewise-linear motion through ``points`` reached at ``times``.

    With ``loop=True`` the motion repeats with period ``times[-1]``; the path
    should end where it starts. ``phase`` shifts the start time (seconds).
    Yaw follows the direction of travel.
    """

    times: tuple[float, ...]
    points: tuple[tuple[float, float], ...]
    loop: bool = True
    phase: float = 0.0
    _headings: tuple[float, ...] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "times", tuple(float(t) for t in self.times))
        object.__setattr__(self, "points", tuple((float(x), float(y)) for x, y in self.points))
        if len(self.times) != len(self.points) or not self.times:
            raise ValueError("times and points must be non-empty and the same length")
        if self.times[0] != 0.0:
            raise ValueError("times must start at 0")
        if any(b <= a for a, b in zip(self.times, self.times[1:])):
            raise ValueError("times must be strictly increasing")
        heads = []
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            heads.append(math.atan2(y1 - y0, x1 - x0) if (x0, y0) != (x1, y1) else (heads[-1] if heads else 0.0))
        object.__setattr__(self, "_headings", tuple(heads) or (0.0,))

    @property
    def period(self) -> float | None:
        return self.times[-1] if self.loop else None

    def pose_at(self, t: float) -> Pose2D:
        t = t + self.phase
        end = self.times[-1]
        if len(self.times) == 1:
            x, y = self.points[0]
            return Pose2D(x, y, 0.0)
        if self.loop and end > 0:
            t = t % end
        t = min(max(t, 0.0), end)
        i = min(bisect.bisect_right(self.times, t) - 1, len(self.times) - 2)
        t0, t1 = self.times[i], self.times[i + 1]
        a = (t - t0) / (t1 - t0)
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        return Pose2D(x0 + a * (x1 - x0), y0 + a * (y1 - y0), self._headings[i])

    def positions(self, times: np.ndarray) -> np.ndarray:
        t = np.asarray(times, dtype=float) + self.phase
        end = self.times[-1]
        if self.loop and end > 0:
            t = np.mod(t, end)
        xs, ys = zip(*self.points)
        return np.stack([np.interp(t, self.times, xs), np.interp(t, self.times, ys)], axis=1)

    def as_waypoints(self, dt: float = 0.1) -> "WaypointTrajectory":
        if self.phase == 0.0:
            return self
        return super().as_waypoints(dt)

    def params(self) -> dict:
        return {
            "times": list(self.times),
            "points": [list(p) for p in self.points],
            "loop": self.loop,
            "phase": self.phase,
        }

    @classmethod
    def from_params(cls, p: dict) -> "WaypointTrajectory":
        return cls(
            times=tuple(p["times"]),
            points=tuple(tuple(q) for q in p["points"]),
            loop=bool(p.get("loop", True)),
            phase=float(p.get("phase", 0.0)),
        )

    @classmethod
    def from_path(
        cls,
        points: Sequence[tuple[float, float]],
        speeds: Sequence[float] | float,
        dwell: float = 0.0,
        loop: bool = True,
        phase: float = 0.0,
    ) -> "WaypointTrajectory":
        """Build timing from a path and per-segment speed(s) in m/s."""
        pts = [tuple(map(float, p)) for p in points]
        if isinstance(speeds, (int, float)):
            speeds = [float(speeds)] * (len(pts) - 1)
        if len(speeds) != len(pts) - 1:
            raise ValueError("need one speed per segment")
        times = [0.0]
        for (x0, y0), (x1, y1), v in zip(pts, pts[1:], speeds):
            times.append(times[-1] + math.hypot(x1 - x0, y1 - y0) / max(v, 1e-6))
        if dwell > 0:
            pts.append(pts[-1])
            times.append(times[-1] + dwell)
        return cls(times=tuple(times), points=tuple(pts), loop=loop, phase=phase)


@TRAJECTORIES.register("path")
@dataclass(frozen=True)
class PathTrajectory(Trajectory):
    """Waypoint path with speed, per-waypoint pauses, optional turning on the
    spot and a start offset, as used by world-collection's moving objects.

    mode:     "loop"   w0 -> w1 -> ... -> w(n-1) -> w0
              "patrol" w0 -> ... -> w(n-1), then back -> ... -> w0
    yaw_mode: "heading" faces along each segment and turns on the spot at
              ``turn_rate`` (rad/s) at waypoints; a patrol drives the return
              pass backwards without turning. "fixed" always faces ``yaw``.

    Each segment is: move, then (heading mode) a turn towards the next
    segment, then the pause at the waypoint reached. Durations are integer
    nanoseconds rounded half-to-even, so the timing matches world-collection's
    Gazebo plugin exactly (rules in its worldgen/moving.py, "Kinematics").
    """

    waypoints: tuple[tuple[float, float], ...]
    speed: float
    dwells: tuple[float, ...] = ()       # seconds per waypoint (default 0)
    mode: str = "loop"
    yaw_mode: str = "heading"
    yaw: float = 0.0                     # used when yaw_mode == "fixed"
    turn_rate: float = 1.0               # rad/s
    phase: float = 0.0                   # seconds the motion is ahead at t = 0
    _phases: tuple = field(init=False, repr=False, compare=False)
    _cycle_ns: int = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        wps = tuple((float(x), float(y)) for x, y in self.waypoints)
        dw = tuple(float(d) for d in self.dwells) or (0.0,) * len(wps)
        object.__setattr__(self, "waypoints", wps)
        object.__setattr__(self, "dwells", dw)
        if len(wps) < 2:
            raise ValueError("a path needs at least 2 waypoints")
        if len(dw) != len(wps):
            raise ValueError("need one dwell per waypoint")
        if self.mode not in ("loop", "patrol"):
            raise ValueError("mode must be 'loop' or 'patrol'")
        if self.yaw_mode not in ("heading", "fixed"):
            raise ValueError("yaw_mode must be 'heading' or 'fixed'")
        if self.speed <= 0 or self.turn_rate <= 0:
            raise ValueError("speed and turn_rate must be > 0")
        self._build()

    @staticmethod
    def _wrap_delta(a: float) -> float:
        d = math.remainder(a, 2 * math.pi)
        return d + 2 * math.pi if d <= -math.pi else d

    def _build(self) -> None:
        w, n = self.waypoints, len(self.waypoints)
        if self.mode == "loop":
            segs = [(i, (i + 1) % n) for i in range(n)]
        else:
            segs = [(i, i + 1) for i in range(n - 1)] + [(i, i - 1) for i in range(n - 1, 0, -1)]

        def seg_yaw(a: int, b: int) -> float:
            if self.yaw_mode == "fixed":
                return self.yaw
            if self.mode == "patrol" and b < a:  # backward pass keeps the forward direction
                a, b = b, a
            return math.atan2(w[b][1] - w[a][1], w[b][0] - w[a][0])

        yaws = [seg_yaw(a, b) for a, b in segs]
        phases = []  # (start_ns, duration_ns, kind, data)
        t = 0
        for k, (a, b) in enumerate(segs):
            dx, dy = w[b][0] - w[a][0], w[b][1] - w[a][1]
            length = math.sqrt(dx * dx + dy * dy)
            yaw_this, yaw_next = yaws[k], yaws[(k + 1) % len(segs)]
            delta = self._wrap_delta(yaw_next - yaw_this) if self.yaw_mode == "heading" else 0.0
            for kind, dur, data in (
                ("move", round(length / self.speed * 1e9), (w[a], w[b], yaw_this)),
                ("turn", round(abs(delta) / self.turn_rate * 1e9) if delta != 0 else 0, (w[b], yaw_this, delta)),
                ("dwell", round(self.dwells[b] * 1e9), (w[b], yaw_next if delta != 0 else yaw_this)),
            ):
                if dur > 0:
                    phases.append((t, dur, kind, data))
                    t += dur
        if t <= 0:
            raise ValueError("path has zero duration")
        object.__setattr__(self, "_phases", tuple(phases))
        object.__setattr__(self, "_cycle_ns", t)

    @property
    def cycle_ns(self) -> int:
        return self._cycle_ns

    @property
    def period(self) -> float | None:
        return self._cycle_ns / 1e9

    def pose_at(self, t: float) -> Pose2D:
        T = self._cycle_ns
        tau = ((round(t * 1e9) % T) + round(self.phase * 1e9)) % T
        starts = [p[0] for p in self._phases]
        start, dur, kind, data = self._phases[bisect.bisect_right(starts, tau) - 1]
        f = float(tau - start) / float(dur)
        if kind == "move":
            (x0, y0), (x1, y1), yaw = data
            return Pose2D(x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw)
        if kind == "turn":
            (x, y), yaw_this, delta = data
            return Pose2D(x, y, yaw_this + f * delta)
        (x, y), yaw = data
        return Pose2D(x, y, yaw)

    def params(self) -> dict:
        return {"waypoints": [list(p) for p in self.waypoints], "speed": self.speed,
                "dwells": list(self.dwells), "mode": self.mode, "yaw_mode": self.yaw_mode,
                "yaw": self.yaw, "turn_rate": self.turn_rate, "phase": self.phase}

    @classmethod
    def from_params(cls, p: dict) -> "PathTrajectory":
        return cls(waypoints=tuple(tuple(q) for q in p["waypoints"]), speed=float(p["speed"]),
                   dwells=tuple(p.get("dwells") or ()), mode=p.get("mode", "loop"),
                   yaw_mode=p.get("yaw_mode", "heading"), yaw=float(p.get("yaw", 0.0)),
                   turn_rate=float(p.get("turn_rate", 1.0)), phase=float(p.get("phase", 0.0)))


@TRAJECTORIES.register("polynomial")
@dataclass(frozen=True)
class PolynomialTrajectory(Trajectory):
    """x(s), y(s) polynomials over s in [0, 1], swept forward then back
    (ping-pong) over ``duration`` seconds each way. Coefficients are in
    increasing order: c0 + c1*s + c2*s^2 + ..."""

    coeffs_x: tuple[float, ...]
    coeffs_y: tuple[float, ...]
    duration: float

    @staticmethod
    def _poly(c: Sequence[float], s: float) -> float:
        return sum(ci * s**i for i, ci in enumerate(c))

    @property
    def period(self) -> float | None:
        return 2.0 * self.duration

    def pose_at(self, t: float) -> Pose2D:
        u = (t % self.period) / self.duration
        forward = u <= 1.0
        s = u if forward else 2.0 - u
        x, y = self._poly(self.coeffs_x, s), self._poly(self.coeffs_y, s)
        ds = 1e-4
        dx = self._poly(self.coeffs_x, min(s + ds, 1.0)) - self._poly(self.coeffs_x, max(s - ds, 0.0))
        dy = self._poly(self.coeffs_y, min(s + ds, 1.0)) - self._poly(self.coeffs_y, max(s - ds, 0.0))
        yaw = math.atan2(dy, dx) + (0.0 if forward else math.pi)
        return Pose2D(x, y, yaw)

    def params(self) -> dict:
        return {"coeffs_x": list(self.coeffs_x), "coeffs_y": list(self.coeffs_y), "duration": self.duration}

    @classmethod
    def from_params(cls, p: dict) -> "PolynomialTrajectory":
        return cls(tuple(p["coeffs_x"]), tuple(p["coeffs_y"]), float(p["duration"]))
