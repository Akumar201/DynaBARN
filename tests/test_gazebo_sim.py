"""Gazebo integration: runs gz-sim headless (skipped when Gazebo is not
installed, e.g. outside the docker image) and checks it against dynabarn's
own trajectories and against MuJoCo.

    docker compose -f docker/compose.yaml run --rm dev pytest tests/test_gazebo_sim.py
"""
import math
import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest

gz_transport = pytest.importorskip("gz.transport13", reason="Gazebo Python bindings not installed")
if shutil.which("gz") is None:
    pytest.skip("gz not on PATH", allow_module_level=True)

from gz.msgs10.odometry_pb2 import Odometry  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402

import dynabarn_core  # noqa: E402
from dynabarn_core import AssetLibrary, DynaBarn, create_backend, create_generator  # noqa: E402

COMMAND = (0.5, 0.3)   # m/s, rad/s, sent from t = 1 s
RUN_FOR = 8.0          # simulated seconds


def _stamp(h) -> float:
    return h.stamp.sec + h.stamp.nsec * 1e-9


@pytest.fixture(scope="module")
def gz_run(tmp_path_factory):
    """Run one lanes scenario in Gazebo, recording obstacle poses and odometry."""
    out = tmp_path_factory.mktemp("gz")
    lib = AssetLibrary()
    scenario = create_generator("lanes", lib, robot_asset="jackal", lanes=3, count=6).build(seed=4)
    art = create_backend("gazebo", lib).export(scenario, out)
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(Path(dynabarn_core.__file__).parents[1]), env.get("PYTHONPATH", "")])
    proc = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(art.scene)], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    poses, odom = [], []
    names = {f"obstacle_{o.id}" for o in scenario.obstacles}

    def on_poses(msg):
        t = _stamp(msg.header)
        poses.append((t, {p.name: (p.position.x, p.position.y) for p in msg.pose if p.name in names}))

    def on_odom(msg):
        q = msg.pose.orientation
        odom.append((_stamp(msg.header), msg.pose.position.x, msg.pose.position.y, 2 * math.atan2(q.z, q.w)))

    node = gz_transport.Node()
    node.subscribe(Pose_V, "/world/dynabarn/pose/info", on_poses)
    node.subscribe(Odometry, "/dynabarn/odom", on_odom)
    pub = node.advertise("/dynabarn/cmd_vel", Twist)
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = COMMAND
    deadline = time.time() + 120
    try:
        while time.time() < deadline and (not odom or odom[-1][0] < RUN_FOR):
            if odom and odom[-1][0] >= 1.0:
                pub.publish(cmd)
            time.sleep(0.02)
    finally:
        proc.terminate()
        log = proc.communicate(timeout=20)[0]
    assert odom and odom[-1][0] >= RUN_FOR, f"Gazebo did not run:\n{log[-3000:]}"
    return {"scenario": scenario, "lib": lib, "poses": poses, "odom": odom, "log": log}


def test_dynabarn_system_loaded(gz_run):
    assert "[dynabarn] 6 moving obstacle(s), odometry: ground_truth" in gz_run["log"]


def test_obstacles_are_exactly_on_their_trajectories(gz_run):
    """Every recorded obstacle pose equals its trajectory at the message's
    timestamp, to float precision."""
    traj = {f"obstacle_{o.id}": o.trajectory for o in gz_run["scenario"].obstacles}
    checked = 0
    worst = 0.0
    for t, objs in gz_run["poses"]:
        if t < 0.5:
            continue
        for name, (x, y) in objs.items():
            p = traj[name].pose_at(t)
            worst = max(worst, math.hypot(x - p.x, y - p.y))
            checked += 1
    assert checked > 1000
    assert worst < 1e-6, f"worst deviation {worst:.2e} m"


def test_odometry_starts_at_zero_and_follows_command(gz_run):
    odom = gz_run["odom"]
    t0, x0, y0, yaw0 = odom[0]
    assert abs(x0) < 1e-3 and abs(y0) < 1e-3 and abs(yaw0) < 1e-3
    seg = [o for o in odom if o[0] >= 3.0]  # past the acceleration
    dt = seg[-1][0] - seg[0][0]
    yaws = [o[3] for o in seg]
    turned = sum((b - a + math.pi) % (2 * math.pi) - math.pi for a, b in zip(yaws, yaws[1:]))
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seg, seg[1:]))
    assert dist / dt == pytest.approx(COMMAND[0], rel=0.05)
    assert turned / dt == pytest.approx(COMMAND[1], rel=0.05)


def test_same_motion_as_mujoco(gz_run):
    """Same scenario and command in MuJoCo: the robot ends up in nearly the same place."""
    pytest.importorskip("mujoco")
    with DynaBarn(gz_run["scenario"], assets=gz_run["lib"], realtime=None) as db:
        while db.time < 1.0:
            db.step()
        while db.time < RUN_FOR:
            db.step(dynabarn_core.VelocityCommand(*COMMAND))
        mj = db.get_data("odom").pose
    gz = min(gz_run["odom"], key=lambda o: abs(o[0] - RUN_FOR))
    assert math.hypot(gz[1] - mj.x, gz[2] - mj.y) < 0.15, (gz, mj)
    assert abs((gz[3] - mj.yaw + math.pi) % (2 * math.pi) - math.pi) < 0.1
