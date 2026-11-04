"""DynaBARN in Gazebo (gz-sim): same scenario arguments, same ROS topics as
mujoco.launch.py.

    ros2 launch dynabarn_ros gazebo.launch.py preset:=dynabarn_lanes seed:=3
    ros2 launch dynabarn_ros gazebo.launch.py headless:=true      # no GUI (tests, CI)

Steps: export the scenario to SDF (dynabarn_core), start gz-sim with
dynabarn's Python system (moving obstacles + odometry), bridge the topics
with ros_gz_bridge (config generated from dynabarn_core.interface) and
publish the static sensor frames.
"""
import json
import os
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    "scenario": ("", "scenario file (instead of preset/generator)"),
    "preset": ("", "preset name, see `dynabarn list presets`"),
    "generator": ("", "generator name (lanes, random, empty); default lanes"),
    "params": ("", "generator overrides as YAML, e.g. \"{lanes: 3, count: 6}\""),
    "seed": ("0", "scenario seed"),
    "assets": ("", "extra asset folders, ':'-separated"),
    "headless": ("false", "run gz-sim without its GUI"),
    "out_dir": ("/tmp/dynabarn_gazebo", "where the exported world goes"),
    "namespace": ("", "ROS namespace for all robot topics"),
}


def _setup(context):
    # Launch files run under the system Python, which may not see dynabarn-core
    # (e.g. when it is installed in a venv), so the export runs as a separate
    # executable, whose interpreter does.
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    exe = os.path.join(get_package_prefix("dynabarn_ros"), "lib", "dynabarn_ros", "prepare_gazebo")
    cmd = [exe, "-o", a["out_dir"], "--json", "--seed", a["seed"]]
    for k in ("scenario", "preset", "generator", "params", "assets"):
        if a[k]:
            cmd += [f"--{k}", a[k]]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"preparing the Gazebo world failed:\n{res.stderr or res.stdout}")
    info = json.loads(res.stdout.strip().splitlines()[-1])
    headless = a["headless"].lower() in ("1", "true", "yes")

    # gz-sim's embedded Python must find dynabarn_core for the dynabarn system
    pythonpath = os.pathsep.join(p for p in (info["core_path"], os.environ.get("PYTHONPATH", "")) if p)
    cmd = ["gz", "sim", "-r"] + (["-s", "--headless-rendering"] if headless else []) + [info["world"]]
    robot = info["robot"]

    ns = a["namespace"]
    return [
        ExecuteProcess(cmd=cmd, output="screen", additional_env={"PYTHONPATH": pythonpath}),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="dynabarn_bridge", namespace=ns,
             parameters=[{"config_file": info["bridge"], "use_sim_time": True}], output="screen"),
        Node(package="dynabarn_ros", executable="sensor_frames", namespace=ns,
             parameters=[{"robot": robot, "assets": a["assets"], "use_sim_time": True}], output="screen"),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=d, description=h) for k, (d, h) in ARGS.items()],
        OpaqueFunction(function=_setup),
    ])
