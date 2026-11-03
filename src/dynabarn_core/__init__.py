"""dynabarn: simulator-independent dynamic-obstacle navigation benchmarks.

Layers (each only depends on the ones above it):
  dynabarn_core.core        dataclasses, trajectories, registries, scenario I/O (no ROS, no simulator)
  dynabarn_core.assets      asset manifests and the asset search path
  dynabarn_core.generators  scenario generators
  dynabarn_core.backends    simulator adapters (mujoco, ...)
  dynabarn_core.runtime     DynaBarn: run in a background thread, get()/set_command()
"""
from .core import Pose2D, Scenario, VelocityCommand
from .assets import AssetLibrary
from .generators import create_generator
from .backends import create_backend
from .runtime import DynaBarn, DynaBarnError

__version__ = "0.1.0"
__all__ = ["DynaBarn", "DynaBarnError", "Pose2D", "Scenario", "VelocityCommand", "AssetLibrary",
           "create_generator", "create_backend", "__version__"]
