"""DynaBARN in MuJoCo.

    ros2 launch dynabarn_ros mujoco.launch.py preset:=dynabarn_lanes seed:=3 viewer:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    "scenario": ("", "scenario file (instead of preset/generator)"),
    "preset": ("", "preset name, see `dynabarn list presets`"),
    "generator": ("", "generator name (lanes, random, empty); default lanes"),
    "params": ("", "generator overrides as YAML, e.g. \"{lanes: 3, count: 6}\""),
    "seed": ("0", "scenario seed"),
    "assets": ("", "extra asset folders, ':'-separated"),
    "enable": ("", "sensors to switch on, comma-separated (e.g. rgb,depth)"),
    "disable": ("", "sensors to switch off, comma-separated"),
    "realtime": ("1.0", "simulation speed vs wall clock (0 = as fast as possible)"),
    "viewer": ("false", "show the MuJoCo window"),
    "follow": ("false", "viewer camera follows the robot"),
    "namespace": ("", "ROS namespace for all robot topics"),
}


def generate_launch_description() -> LaunchDescription:
    c = {k: LaunchConfiguration(k) for k in ARGS}
    params = {k: c[k] for k in ("scenario", "preset", "generator", "params", "assets", "enable", "disable")}
    params["seed"] = ParameterValue(c["seed"], value_type=int)
    params["realtime"] = ParameterValue(c["realtime"], value_type=float)
    params["viewer"] = ParameterValue(c["viewer"], value_type=bool)
    params["follow"] = ParameterValue(c["follow"], value_type=bool)
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=d, description=h) for k, (d, h) in ARGS.items()],
        Node(package="dynabarn_ros", executable="mujoco_node", namespace=c["namespace"],
             parameters=[params], output="screen", emulate_tty=True),
    ])
