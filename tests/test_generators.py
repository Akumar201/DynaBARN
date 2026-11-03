import numpy as np
import pytest

from dynabarn_core.generators import GENERATORS, create_generator


@pytest.mark.parametrize("name", ["lanes", "random"])
def test_same_seed_same_scenario(name):
    a = create_generator(name, count=10).build(seed=3)
    b = create_generator(name, count=10).build(seed=3)
    assert a.to_dict() == b.to_dict()


def test_random_different_seeds_differ():
    a = create_generator("random", count=5).build(seed=1)
    b = create_generator("random", count=5).build(seed=2)
    assert a.to_dict()["obstacles"] != b.to_dict()["obstacles"]


@pytest.mark.parametrize(
    "name,params",
    [
        ("lanes", {"axis": "row"}),
        ("lanes", {"axis": "column", "speed_mode": "segment", "speed_min": 0.5, "speed_max": 1.5}),
        ("random", {"bounce": True, "speed_mode": "agent", "speed_min": 0.5, "speed_max": 1.5}),
    ],
)
def test_obstacles_stay_in_bounds(name, params):
    s = create_generator(name, count=12, **params).build(seed=5)
    assert len(s.obstacles) == 12
    for o in s.obstacles:
        for k in range(200):
            p = o.trajectory.pose_at(k * 0.37)
            assert s.bounds.contains(p.x, p.y, margin=1e-6)


def _closest_surface_gap(gen, scenario, horizon=120.0):
    r = gen.obstacle_radius
    times = np.arange(0.0, horizon, 0.1)
    tracks = [o.trajectory.positions(times) for o in scenario.obstacles]
    worst = np.inf
    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            worst = min(worst, np.min(np.linalg.norm(tracks[i] - tracks[j], axis=1)) - 2 * r)
    return worst


@pytest.mark.parametrize(
    "name,params",
    [
        ("lanes", {"count": 36, "lanes": 6}),
        ("lanes", {"count": 12, "lanes": 3, "stagger_phase": True}),
        ("lanes", {"count": 24, "lanes": 8, "axis": "column"}),
        ("lanes", {"count": 36, "speed_mode": "agent", "speed_min": 0.4, "speed_max": 2.0}),
        ("lanes", {"count": 36, "obstacle_asset": "box_crate"}),
        ("random", {"count": 36}),
        ("random", {"count": 60, "bounce": True, "points": 30}),
        ("random", {"count": 36, "speed_mode": "segment", "speed_min": 0.4, "speed_max": 2.0}),
        ("random", {"count": 4, "layout": "free"}),
    ],
)
def test_obstacles_never_overlap(name, params):
    """Obstacle surfaces must stay min_clearance (default 0) apart."""
    gen = create_generator(name, **params)
    assert _closest_surface_gap(gen, gen.build(seed=0)) >= -1e-9


def test_lane_gap_is_surface_clearance():
    gen = create_generator("lanes", count=12, lanes=1, lane_gap=0.5, stagger_phase=True, min_clearance=None)
    assert _closest_surface_gap(gen, gen.build(seed=0)) == pytest.approx(0.5, abs=0.02)


def test_min_clearance_is_respected():
    gen = create_generator("random", count=36, min_clearance=0.6)
    assert _closest_surface_gap(gen, gen.build(seed=3)) >= 0.6 - 1e-9


def test_free_layout_reports_when_it_cannot_fit():
    with pytest.raises(ValueError, match="layout=cells"):
        create_generator("random", count=40, layout="free").build(seed=0)
    # explicitly allowing overlaps always works
    assert len(create_generator("random", count=40, layout="free", min_clearance=None).build(seed=0).obstacles) == 40


def test_lanes_rejects_overfull_lane():
    with pytest.raises(ValueError, match="has room for 10 of its 30"):
        create_generator("lanes", count=60, lanes=2, lane_gap=1.0).build(seed=0)


def test_options_from_strings_and_unknown_option():
    g = create_generator("lanes", count="4", stagger_phase="true", speed_min="none")
    assert g.config.count == 4 and g.config.stagger_phase is True and g.config.speed_min is None
    with pytest.raises(TypeError, match="no option 'colour'"):
        create_generator("lanes", colour="red")
    with pytest.raises(ValueError, match="speed_min"):
        create_generator("lanes", speed_mode="agent")


def test_registry_lists_builtins():
    assert {"lanes", "random"} <= set(GENERATORS.names())


def test_presets_build():
    from dynabarn_core.generators import generator_from_preset, list_presets

    names = list_presets()
    assert {"dynabarn_lanes", "dynabarn_random", "burger_lanes", "burger_random"} <= set(names)
    for name in names:
        gen = generator_from_preset(name)
        s = gen.build(seed=0)
        assert len(s.obstacles) > 0
        assert _closest_surface_gap(gen, s, horizon=60) >= -1e-9
    assert generator_from_preset("burger_lanes", count=6).config.count == 6  # override
