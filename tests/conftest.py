import pytest

from dynabarn_core.core import Bounds, ObstacleSpec, Pose2D, RobotSpec, Scenario, StaticTrajectory


def make_scenario(obstacles=(), start=Pose2D(0.0, 0.0, 0.0), name="test"):
    return Scenario(
        name=name,
        seed=0,
        environment="arena_20x20",
        bounds=Bounds(-9, 9, -8, 8),
        robot=RobotSpec("tb3_burger_cam", start),
        goal=Pose2D(10, 0),
        obstacles=tuple(obstacles),
    )


def static_obstacle(oid, x, y, asset="cylinder_red"):
    return ObstacleSpec(oid, asset, StaticTrajectory(Pose2D(x, y)))


@pytest.fixture
def scenario_factory():
    return make_scenario
