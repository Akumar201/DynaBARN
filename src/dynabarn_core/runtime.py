"""``DynaBarn``: run a scenario in a background thread and read/write it from
anywhere (Facade over scenario + backend + simulation).

    db = DynaBarn.from_preset("burger_lanes", seed=1)
    db.start()                      # physics runs in real time, in a thread
    db.get("imu")                   # latest reading (SensorReading) of any sensor
    db.set_command(0.2, 0.0)        # m/s, rad/s
    db.stop()

Getters never block the simulation and can be called at any rate from any
thread (e.g. ROS 2 timers or subscriptions). The thread advances physics
in slices no longer than the fastest enabled sensor's period, so a 200 Hz
IMU really delivers a new sample every 5 ms of wall time.

Without ``start()`` the same object can be stepped synchronously with
``step()``, e.g. for reinforcement learning or tests.
"""
from __future__ import annotations

import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable

from .assets import AssetLibrary
from .backends import Artifacts, BackendError, Simulation, create_backend
from .core.messages import Observation, SensorInfo, SensorReading, VelocityCommand
from .core.scenario import Scenario


class DynaBarnError(RuntimeError):
    pass


class DynaBarn:
    def __init__(
        self,
        scenario: Scenario | str | Path,
        backend: str = "mujoco",
        assets: AssetLibrary | None = None,
        out_dir: str | Path | None = None,
        realtime: float | None = 1.0,
        command_timeout: float | None = None,
        **sim_options: Any,
    ) -> None:
        """
        scenario:        a Scenario or a path to a scenario file
        realtime:        simulation speed vs wall clock when running in the
                         background (1.0 = real time, 2.0 = twice as fast,
                         None = as fast as possible)
        command_timeout: stop the robot if no command arrives for this many
                         wall-clock seconds (like a cmd_vel watchdog); None = never
        sim_options:     passed to the backend, e.g. control_hz, seed,
                         enable=["rgb"], disable=["imu"]
        """
        self.scenario = scenario if isinstance(scenario, Scenario) else Scenario.load(scenario)
        self.assets = assets or AssetLibrary()
        self.backend = create_backend(backend, self.assets)
        if not self.backend.capabilities.in_process:
            raise DynaBarnError(f"backend '{backend}' runs out of process; DynaBarn needs an in-process backend")
        out = Path(out_dir) if out_dir else Path(tempfile.mkdtemp(prefix=f"dynabarn_{backend}_"))
        self.artifacts: Artifacts = self.backend.export(self.scenario, out)
        self.sim: Simulation = self.backend.create_simulation(self.artifacts, **sim_options)
        self.realtime = realtime
        self.command_timeout = command_timeout

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._new_data = threading.Condition()
        self._error: BaseException | None = None
        self._last_command_wall = time.monotonic()
        self._measured_rtf = 0.0

    # -- construction helpers ----------------------------------------------------
    @classmethod
    def from_preset(cls, preset: str, seed: int = 0, assets: AssetLibrary | None = None,
                    params: dict | None = None, **kwargs: Any) -> "DynaBarn":
        """Generate a scenario from a preset (with optional ``params``
        overrides) and wrap it. ``kwargs`` go to ``DynaBarn(...)``."""
        from .generators import generator_from_preset

        scenario = generator_from_preset(preset, assets=assets, **(params or {})).build(seed=seed, name=preset)
        return cls(scenario, assets=assets, **kwargs)

    @classmethod
    def from_generator(cls, generator: str, seed: int = 0, assets: AssetLibrary | None = None,
                       params: dict | None = None, **kwargs: Any) -> "DynaBarn":
        from .generators import create_generator

        scenario = create_generator(generator, assets=assets, **(params or {})).build(seed=seed)
        return cls(scenario, assets=assets, **kwargs)

    # -- lifecycle -----------------------------------------------------------------
    def start(self) -> "DynaBarn":
        """Run the simulation in a background thread."""
        if self.running:
            return self
        self._raise_if_failed()
        self._stop.clear()
        self._paused.clear()
        self._last_command_wall = time.monotonic()
        # OpenGL contexts are per thread: drop ours so the sim thread makes its own
        self.sim.release_thread_resources()
        self._thread = threading.Thread(target=self._run, name="dynabarn-core", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._paused.clear()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        with self._new_data:
            self._new_data.notify_all()
        self._raise_if_failed()

    def pause(self) -> None:
        self._paused.set()

    def resume(self) -> None:
        self._paused.clear()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def paused(self) -> bool:
        return self._paused.is_set()

    def reset(self) -> Observation:
        """Restart the episode (also while running)."""
        return self.sim.reset()

    def step(self, command: VelocityCommand | None = None) -> Observation:
        """Synchronous stepping; only when the background thread is not running."""
        if self.running:
            raise DynaBarnError("step() is for synchronous use; call stop() first or use set_command()")
        return self.sim.step(command)

    def close(self) -> None:
        self.stop()
        self.sim.close()

    def __enter__(self) -> "DynaBarn":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- getters ---------------------------------------------------------------------
    def get(self, name: str) -> SensorReading | None:
        """Latest reading of sensor ``name`` (``.data`` holds the payload,
        ``.stamp`` the simulation time), or None before the first one."""
        self._raise_if_failed()
        return self.sim.latest(name)

    def get_data(self, name: str) -> Any:
        """Payload of the latest reading (e.g. an ``Imu``), or None."""
        r = self.get(name)
        return r.data if r is not None else None

    def wait_for(self, name: str, newer_than: float | None = None, timeout: float | None = 1.0) -> SensorReading | None:
        """Block until sensor ``name`` has a reading stamped after ``newer_than``
        (default: the current latest one). Returns None on timeout."""
        last = self.sim.latest(name)
        after = newer_than if newer_than is not None else (last.stamp if last else -1.0)
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._new_data:
            while True:
                self._raise_if_failed()
                r = self.sim.latest(name)
                if r is not None and r.stamp > after:
                    return r
                if self._stop.is_set() and not self.running:
                    return None
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._new_data.wait(remaining)

    def observe(self) -> Observation:
        with self.sim._lock:
            return self.sim.observe()

    def sensors(self) -> dict[str, SensorInfo]:
        return self.sim.sensors()

    @property
    def time(self) -> float:
        """Simulation time in seconds."""
        return self.sim.time

    @property
    def realtime_factor(self) -> float:
        """Measured simulation speed vs wall clock while running."""
        return self._measured_rtf

    # -- setters -----------------------------------------------------------------------
    def set_command(self, linear: float | VelocityCommand = 0.0, angular: float = 0.0) -> None:
        cmd = linear if isinstance(linear, VelocityCommand) else VelocityCommand(float(linear), float(angular))
        self._last_command_wall = time.monotonic()
        self.sim.set_command(cmd)

    def set_sensor_enabled(self, name: str, enabled: bool) -> SensorInfo:
        return self.sim.set_sensor_enabled(name, enabled)

    def configure_sensor(self, name: str, **changes: Any) -> SensorInfo:
        """E.g. ``configure_sensor("lidar", noise_std=0.02, rate_hz=10)``."""
        return self.sim.configure_sensor(name, **changes)

    def subscribe(self, name: str, callback: Callable[[SensorReading], None]) -> Callable[[], None]:
        """Callback for every new reading (runs on the simulation thread)."""
        return self.sim.subscribe(name, callback)

    # -- background thread -----------------------------------------------------------
    def _tick(self) -> float:
        """Advance in slices no longer than the fastest enabled sensor period."""
        rates = [s.rate_hz for s in self.sim.sensors().values() if s.enabled]
        tick = min([self.sim.dt] + [1.0 / r for r in rates])
        return max(tick, 1e-3)

    def _run(self) -> None:
        try:
            wall0, sim0 = time.monotonic(), self.sim.time
            while not self._stop.is_set():
                if self._paused.is_set():
                    time.sleep(0.01)
                    wall0, sim0 = time.monotonic(), self.sim.time
                    continue
                if (self.command_timeout is not None
                        and time.monotonic() - self._last_command_wall > self.command_timeout):
                    self.sim.set_command(VelocityCommand())
                self.sim.advance(self._tick())
                with self._new_data:
                    self._new_data.notify_all()
                elapsed_sim = self.sim.time - sim0
                elapsed_wall = time.monotonic() - wall0
                if elapsed_wall > 0:
                    self._measured_rtf = elapsed_sim / elapsed_wall
                if self.realtime:
                    ahead = elapsed_sim / self.realtime - elapsed_wall
                    if ahead > 0:
                        time.sleep(ahead)
                    elif ahead < -0.5:  # fell far behind: don't try to catch up in a burst
                        wall0, sim0 = time.monotonic(), self.sim.time
        except BaseException as e:  # surface errors to the caller's thread
            self._error = e
            with self._new_data:
                self._new_data.notify_all()
        finally:
            self.sim.release_thread_resources()

    def _raise_if_failed(self) -> None:
        if self._error is not None:
            err, self._error = self._error, None
            raise DynaBarnError(f"simulation thread failed: {err!r}") from err


__all__ = ["DynaBarn", "DynaBarnError", "BackendError"]
