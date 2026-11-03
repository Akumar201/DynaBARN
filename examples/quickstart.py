"""Using dynabarn from your own code after `pip install dynabarn-core`.

Run:  python quickstart.py            (headless, prints a summary)
      python quickstart.py --view     (opens the MuJoCo viewer)
"""
import sys
import tempfile

from dynabarn_core import AssetLibrary, VelocityCommand, create_backend, create_generator
from dynabarn_core.assets import EnvironmentAsset, ObstacleAsset, Wall
from dynabarn_core.core import Box, Cylinder
from dynabarn_core.generators import generator_from_preset

# 1. Assets: the built-in ones ship with the package. Add your own in code
#    (no files needed), from a dict, or from a folder of asset.yaml files.
assets = AssetLibrary()
assets.add(
    ObstacleAsset(id="green_barrel", geometry=Cylinder(radius=0.3, length=0.9), color=(0.1, 0.6, 0.2, 1.0)),
    ObstacleAsset(id="pallet", geometry=Box((1.2, 0.8, 0.15)), color=(0.6, 0.45, 0.25, 1.0)),
    EnvironmentAsset(
        id="corridor",
        ground_size=(30, 12),
        walls=(
            Wall("left", 0.0, 5.0, 0.0, (24, 0.2, 1.0)),
            Wall("right", 0.0, -5.0, 0.0, (24, 0.2, 1.0)),
            Wall("back", -12.0, 0.0, 0.0, (0.2, 10, 1.0)),
        ),
    ),
)
# assets.add_folder("my_assets")                      # folder of asset.yaml files
# assets.add_manifest({"id": "cone", "kind": "obstacle",
#                      "geometry": {"type": "sphere", "radius": 0.2}})

# 2. Scenario: a generator (or a preset) plus any options.
scenario = create_generator(
    "lanes", assets=assets,
    environment="corridor", obstacle_asset="green_barrel",
    x_min=-8, x_max=8, y_min=-3.5, y_max=3.5, lanes=4, count=12,
    start_x=-11.3, goal_x=11, speed=0.8, stagger_phase=True,
).build(seed=42, name="corridor_demo")
# scenario = generator_from_preset("burger_lanes", assets=assets).build(seed=1)
scenario.save("corridor_demo.yaml")  # reproducible: share this file

# 3. Simulate.
backend = create_backend("mujoco", assets)
artifacts = backend.export(scenario, tempfile.mkdtemp(prefix="dynabarn_"))

if "--view" in sys.argv:
    backend.view(artifacts, linear=0.5)
    sys.exit()

with backend.create_simulation(artifacts, control_hz=20) as sim:
    obs = sim.reset()
    while obs.time < 60:
        # your controller goes here: obs.pose, obs.velocity, obs.scan, obs.contacts
        front = min(obs.scan.ranges[300:420])  # Jackal lidar: 720 rays over 270 deg
        cmd = VelocityCommand(0.0 if front < 0.8 else 0.6, 0.0)
        obs = sim.step(cmd)
        if obs.contacts:
            print(f"collision at t={obs.time:.1f}s with {obs.contacts[0].other}")
            break
        if obs.pose.distance_to(scenario.goal) < 0.5:
            print(f"goal reached at t={obs.time:.1f}s")
            break
    print(f"final pose: x={obs.pose.x:.2f} y={obs.pose.y:.2f}")
