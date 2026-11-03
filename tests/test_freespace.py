"""Free-space queries and obstacle placement inside maps."""
import math

import numpy as np
import pytest

from dynabarn_core import AssetLibrary, create_generator
from dynabarn_core.assets import EnvironmentAsset, MovingObject, StaticObject
from dynabarn_core.core import Bounds, Box, Cylinder, ExtrudedPolygon, PathTrajectory
from dynabarn_core.generators.freespace import FreeSpace

L_SHAPE = ((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4))


def room(objects=(), free_space=(), moving=()):
    """20 x 20 m room with optional extra objects."""
    walls = (
        StaticObject("north", Box((20, 0.2, 1)), 0, 10, 0.5),
        StaticObject("south", Box((20, 0.2, 1)), 0, -10, 0.5),
        StaticObject("east", Box((0.2, 20, 1)), 10, 0, 0.5),
        StaticObject("west", Box((0.2, 20, 1)), -10, 0, 0.5),
    )
    return EnvironmentAsset(id="room", ground_size=(24, 24), objects=walls + tuple(objects),
                            free_space=free_space, moving_objects=tuple(moving))


# ---- FreeSpace -----------------------------------------------------------------
def test_clearance_and_is_free():
    env = room([StaticObject("pillar", Cylinder(0.5, 1), 3, 0, 0.5),
                StaticObject("L", ExtrudedPolygon(L_SHAPE, 2), -6, -6, 0)])
    fs = FreeSpace(env, Bounds(-10, 10, -10, 10))
    d = fs.clearance([(0, 0), (3, 0), (4.5, 0), (-5, -5), (-3, -3), (0, 9.5)])
    # (-3,-3) is in the L's notch: 1 m from both notch walls (they extend past the inner corner)
    assert d == pytest.approx([3 - 0.5 - 0, 0, 1.0, 0, 1.0, 0.4], abs=1e-9)
    # inside the L's notch (world x,y in [-4,-2]) is free; inside its arms is not
    assert list(fs.is_free([(-3, -3), (-5, -5), (-3, -5.5)], 0.1)) == [True, False, False]
    assert not fs.is_free([(3.0, 0.0)], 0.0)[0]          # strictly inside: never free
    assert fs.is_free([(4.6, 0.0)], 1.0)[0] is np.True_  # 1.1 m from the pillar


def test_region_outline_even_odd_and_keep_clear():
    outer = ((-8, -8), (8, -8), (8, 8), (-8, 8))
    hole = ((-2, -2), (2, -2), (2, 2), (-2, 2))
    fs = FreeSpace(room(free_space=(outer, hole)), Bounds(-10, 10, -10, 10), keep_clear=[(5, 5, 1.0)])
    assert list(fs.in_region([(0, 0), (5, 0), (9, 0)])) == [False, True, False]
    assert list(fs.is_free([(5, 5.9), (5, 6.6)], 0.5)) == [False, True]  # keep-clear circle r=1 at (5, 5)


def test_free_runs_and_paths():
    env = room([StaticObject("divider", Box((0.2, 4, 1)), 0, 0, 0.5)])
    fs = FreeSpace(env, Bounds(-9, 9, -9, 9))
    runs = fs.free_runs((-9, 0), (9, 0), 0.3)
    assert len(runs) == 2
    assert runs[0][0] == pytest.approx(0.0) and runs[0][1] == pytest.approx(9 - 0.1 - 0.3, abs=0.06)
    assert runs[1][0] == pytest.approx(9 + 0.1 + 0.3, abs=0.06) and runs[1][1] == pytest.approx(18.0)
    assert not fs.path_free([(-5, 0), (5, 0)], 0.3)
    assert fs.path_free([(-5, 3), (5, 3)], 0.3)


# ---- generators in maps ----------------------------------------------------------
def _lib(env):
    return AssetLibrary(include_env=False).add(env)


def _min_static_gap(fs, scenario, r, times=np.arange(0, 120, 0.1)):
    return min(float(fs.clearance(o.trajectory.positions(times)).min()) - r for o in scenario.obstacles)


@pytest.mark.parametrize("gen,params", [
    ("lanes", dict(lanes=5, count=10, stagger_phase=True)),
    ("lanes", dict(axis="column", lanes=4, count=8)),
    ("random", dict(count=20)),
    ("random", dict(count=20, bounce=True, points=25)),
])
def test_obstacles_avoid_walls_and_start(gen, params):
    env = room([StaticObject("divider", Box((0.2, 12, 1)), 0, 0, 0.5),
                StaticObject("pillar", Cylinder(0.8, 1), -5, 4, 0.5),
                StaticObject("L", ExtrudedPolygon(L_SHAPE, 2), 3, -8, 0)])
    lib = _lib(env)
    g = create_generator(gen, assets=lib, environment="room", robot_asset="jackal", area="environment",
                         start_x=-8.5, start_y=-8.5, goal_x=8.5, goal_y=8.5, **params)
    s = g.build(seed=4)
    assert len(s.obstacles) == params["count"]
    walls_only = FreeSpace(env, Bounds(-10, 10, -10, 10))
    assert _min_static_gap(walls_only, s, g.obstacle_radius) >= g.config.static_clearance - 1e-6
    times = np.arange(0, 120, 0.1)
    keep = lib.robot("jackal").footprint_radius + g.config.keep_clear
    for o in s.obstacles:
        xy = o.trajectory.positions(times)
        for px, py in ((-8.5, -8.5), (8.5, 8.5)):
            assert np.min(np.hypot(xy[:, 0] - px, xy[:, 1] - py)) - g.obstacle_radius >= keep - 1e-6


def test_lanes_split_around_a_wall():
    env = room([StaticObject("divider", Box((0.2, 20, 1)), 0, 0, 0.5)])  # blocks every lane at x = 0
    g = create_generator("lanes", assets=_lib(env), environment="room", area="environment", lanes=3, count=6,
                         start_x=-9.5, start_y=-9.5)
    s = g.build(seed=0)
    xs = [sorted(p[0] for p in o.trajectory.points[:2]) for o in s.obstacles]
    assert sum(1 for a, b in xs if b < 0) == 3 and sum(1 for a, b in xs if a > 0) == 3  # 3 per side


def test_lanes_report_when_free_space_is_too_short():
    env = room([StaticObject(f"d{k}", Box((0.2, 20, 1)), x, 0, 0.5) for k, x in enumerate((-5, 0, 5))])
    with pytest.raises(ValueError, match="has room for"):
        create_generator("lanes", assets=_lib(env), environment="room", area="environment",
                         lanes=1, count=40).build(seed=0)


def test_random_avoids_environment_moving_objects():
    # a cart sweeping the middle of the room
    cart = MovingObject("cart", Box((1.2, 0.6, 1)), 0.5,
                        PathTrajectory(waypoints=((-7, 0), (7, 0)), speed=1.0, mode="patrol"))
    env = room(moving=[cart])
    g = create_generator("random", assets=_lib(env), environment="room", area="environment", count=16,
                         start_x=-9, start_y=-9)
    s = g.build(seed=2)
    times = np.arange(0, 120, 0.1)
    c = cart.trajectory.positions(times)
    for o in s.obstacles:
        gap = np.min(np.linalg.norm(o.trajectory.positions(times) - c, axis=1)) - g.obstacle_radius \
            - cart.shape.footprint_radius
        assert gap >= -1e-9
    # without the environment's movers the same placement may use the middle
    g2 = create_generator("random", assets=_lib(env), environment="room", area="environment", count=16,
                          start_x=-9, start_y=-9, environment_obstacles=False)
    assert g2.build(seed=2).environment_obstacles is False
