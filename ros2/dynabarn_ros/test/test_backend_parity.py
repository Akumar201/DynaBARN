"""Both launch files must give a ROS user the same robot.

For MuJoCo and Gazebo (headless) this checks topics + types, sensor rates,
frames, the TF tree, simulation time and the reaction to cmd_vel, then
compares the two. Needs a sourced ROS 2 workspace with dynabarn_ros and
Gazebo (the docker image has both):

    docker compose -f docker/compose.yaml run --rm dev \\
        python -m pytest /work/ros2/dynabarn_ros/test -v
"""
import math
import os
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

# empty arena: the cmd_vel check must not be disturbed by obstacles hitting the robot
LAUNCH_ARGS = ["generator:=empty", "seed:=1"]
BACKENDS = {
    "mujoco": ["ros2", "launch", "dynabarn_ros", "mujoco.launch.py", *LAUNCH_ARGS],
    "gazebo": ["ros2", "launch", "dynabarn_ros", "gazebo.launch.py", "headless:=true", *LAUNCH_ARGS],
}
EXPECTED_RATES = {"scan": 40.0, "imu": 200.0, "odom": 50.0}  # jackal asset.yaml
CMD = (0.5, 0.3)


def _measure(name: str) -> dict:
    proc = subprocess.Popen(BACKENDS[name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    node = rclpy.create_node(f"parity_{name}")
    data = {"stamps": {k: [] for k in EXPECTED_RATES}, "frames": {}, "odom": [], "clock": [], "static": {}}

    def rec(key):
        def cb(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data["stamps"][key].append(t)
            data["frames"][key] = msg.header.frame_id
            if key == "odom":
                q = msg.pose.pose.orientation
                data["odom"].append((t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                                     2 * math.atan2(q.z, q.w), msg.child_frame_id))
        return cb

    node.create_subscription(LaserScan, "scan", rec("scan"), 50)
    node.create_subscription(Imu, "imu", rec("imu"), 500)
    node.create_subscription(Odometry, "odom", rec("odom"), 100)
    node.create_subscription(Clock, "/clock", lambda m: data["clock"].append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
    static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_static(msg):
        for tf in msg.transforms:
            data["static"][tf.child_frame_id] = (tf.header.frame_id, round(tf.transform.translation.x, 4),
                                                 round(tf.transform.translation.y, 4), round(tf.transform.translation.z, 4))

    node.create_subscription(TFMessage, "/tf_static", on_static, static_qos)
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline and len(data["odom"]) < 5:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert data["odom"], f"{name}: no odometry within 90 s"
        # topics as seen from outside
        data["topics"] = {n: tuple(t) for n, t in node.get_topic_names_and_types()
                          if not n.startswith(("/parameter_events", "/rosout"))}
        for k in data["stamps"]:
            data["stamps"][k].clear()
        t_cmd = data["odom"][-1][0]
        msg = Twist()
        msg.linear.x, msg.angular.z = CMD
        while data["odom"][-1][0] < t_cmd + 6.0 and time.monotonic() < deadline + 60:
            pub.publish(msg)
            end = time.monotonic() + 0.05
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
        data["t_cmd"] = t_cmd
        return data
    finally:
        node.destroy_node()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


@pytest.fixture(scope="module")
def results():
    rclpy.init()
    try:
        return {name: _measure(name) for name in BACKENDS}
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("name", list(BACKENDS))
def test_sensor_rates_in_simulation_time(results, name):
    for key, rate in EXPECTED_RATES.items():
        s = results[name]["stamps"][key]
        assert len(s) > 10, f"{name}/{key}: too few messages"
        measured = (len(s) - 1) / (s[-1] - s[0])
        assert measured == pytest.approx(rate, rel=0.05), f"{name}/{key}"


@pytest.mark.parametrize("name", list(BACKENDS))
def test_robot_follows_cmd_vel(results, name):
    r = results[name]
    seg = [o for o in r["odom"] if o[0] >= r["t_cmd"] + 2.0]
    dt = seg[-1][0] - seg[0][0]
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seg, seg[1:]))
    turned = sum((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi for a, b in zip(seg, seg[1:]))
    assert dist / dt == pytest.approx(CMD[0], rel=0.06), name
    assert turned / dt == pytest.approx(CMD[1], rel=0.06), name


def test_same_topics_and_types(results):
    mj, gz = results["mujoco"]["topics"], results["gazebo"]["topics"]
    assert mj == gz
    for t in ("/scan", "/imu", "/odom", "/joint_states", "/ground_truth", "/clock", "/cmd_vel", "/tf", "/tf_static"):
        assert t in mj


def test_same_frames_and_tf(results):
    mj, gz = results["mujoco"], results["gazebo"]
    assert mj["frames"] == gz["frames"] == {"scan": "base_scan", "imu": "imu_link", "odom": "odom"}
    assert mj["odom"][-1][4] == gz["odom"][-1][4] == "base_link"
    assert mj["static"] == gz["static"]
    assert mj["static"]["base_scan"] == ("base_link", 0.12, 0.0, 0.27)


def test_clock_is_simulation_time(results):
    for name, r in results.items():
        c = r["clock"]
        assert c and all(b >= a for a, b in zip(c, c[1:])), name
        assert c[-1] < 1000, f"{name}: /clock looks like wall time"
