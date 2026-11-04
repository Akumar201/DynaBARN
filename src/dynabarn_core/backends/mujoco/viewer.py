"""Interactive on-screen viewer for the MuJoCo backend.

Keys: Up/Down change forward speed, Left/Right change turn rate, Space stops,
Backspace resets the episode. The mouse controls the camera as in MuJoCo's
own viewer.
"""
from __future__ import annotations

import time

import mujoco
import mujoco.viewer

from ...core.messages import VelocityCommand
from .sim import MujocoSimulation

KEY_SPACE, KEY_BACKSPACE = 32, 259
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265


def run_viewer(sim: MujocoSimulation, linear: float = 0.0, angular: float = 0.0,
               follow: bool = False, duration: float | None = None) -> None:
    drive = sim.robot.drive
    cmd = {"v": linear, "w": angular, "reset": False}
    lin_step, ang_step = drive.max_linear / 4, drive.max_angular / 6

    def on_key(key: int) -> None:
        if key == KEY_UP:
            cmd["v"] = min(cmd["v"] + lin_step, drive.max_linear)
        elif key == KEY_DOWN:
            cmd["v"] = max(cmd["v"] - lin_step, -drive.max_linear)
        elif key == KEY_LEFT:
            cmd["w"] = min(cmd["w"] + ang_step, drive.max_angular)
        elif key == KEY_RIGHT:
            cmd["w"] = max(cmd["w"] - ang_step, -drive.max_angular)
        elif key == KEY_SPACE:
            cmd["v"] = cmd["w"] = 0.0
        elif key == KEY_BACKSPACE:
            cmd["reset"] = True

    v = mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key)
    try:
        _setup_camera(v, sim, follow)
        _loop(v, sim, cmd, duration)
    finally:
        _close(v)


class ViewerHandle:
    """A passive MuJoCo window that someone else steps (see Simulation.open_viewer)."""

    def __init__(self, v) -> None:
        self._v = v

    def sync(self) -> None:
        self._v.sync()

    def is_running(self) -> bool:
        return self._v.is_running()

    def close(self) -> None:
        _close(self._v)


def _close(v) -> None:
    # The viewer renders on its own thread. Exiting Python while that
    # thread is still tearing down its X11 window segfaults inside libX11,
    # so close the window and wait for the thread to finish first.
    v.close()
    deadline = time.monotonic() + 5.0
    while v.is_running() and time.monotonic() < deadline:
        time.sleep(0.02)
    time.sleep(0.3)


def _setup_camera(v, sim: MujocoSimulation, follow: bool) -> None:
    if follow:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = sim._root
        v.cam.distance = max(1.5, 12 * sim.robot.footprint_radius)
        v.cam.elevation, v.cam.azimuth = -25.0, 180.0
    else:
        b = sim.scenario.bounds
        v.cam.lookat[:] = ((b.x_min + b.x_max) / 2, (b.y_min + b.y_max) / 2, 0.0)
        v.cam.distance = 1.35 * max(b.x_max - b.x_min, b.y_max - b.y_min)
        v.cam.elevation, v.cam.azimuth = -55.0, 90.0


def _loop(v, sim: MujocoSimulation, cmd: dict, duration: float | None) -> None:
    hits: set[str] = set()
    obs = sim.observe()
    while v.is_running() and (duration is None or obs.time < duration):
        t0 = time.perf_counter()
        with v.lock():
            if cmd["reset"]:
                cmd.update(v=0.0, w=0.0, reset=False)
                hits.clear()
                obs = sim.reset()
            obs = sim.step(VelocityCommand(cmd["v"], cmd["w"]))
        for c in obs.contacts:
            if c.other not in hits:
                hits.add(c.other)
                print(f"[t={obs.time:6.2f}s] collision with {c.other}", flush=True)
        if hasattr(v, "set_texts"):  # MuJoCo >= 3.2
            try:
                closest = min(obs.scan.ranges) if obs.scan else float("nan")
                v.set_texts((
                    mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "scenario\nrobot\ntime\ncmd v / w\npose\nclosest obstacle\ncollisions",
                    f"{sim.scenario.name}\n{sim.robot.id}\n{obs.time:.1f} s\n"
                    f"{cmd['v']:.2f} m/s / {cmd['w']:.2f} rad/s\n"
                    f"({obs.pose.x:.2f}, {obs.pose.y:.2f})\n{closest:.2f} m\n{len(hits)}",
                ))
            except Exception:
                pass
        v.sync()
        time.sleep(max(0.0, sim.dt - (time.perf_counter() - t0)))
