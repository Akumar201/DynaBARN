"""Run one navigation trial: Gazebo world + Jackal + (optional) Nav2 + trial runner.

Example:
  ros2 launch dynabarn2 run_trial.launch.py world_file:=dynabarn/world_0 planner:=mppi
"""
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('world_file', default_value='dynabarn/world_0',
                          description='World under share/dynabarn2/worlds, without .sdf'),
    DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('planner', default_value='mppi', choices=['mppi', 'dwb', 'none'],
                          description='Baseline Nav2 controller; none leaves the robot to you'),
    DeclareLaunchArgument('nav_log_level', default_value='warn'),
    DeclareLaunchArgument('out_file', default_value='results.csv'),
    DeclareLaunchArgument('timeout', default_value='100', description='Trial timeout (s)'),
    # Start/goal of the DynaBARN task, same as the BARN Challenge's dynamic worlds
    DeclareLaunchArgument('x', default_value='11.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='3.14159'),
    DeclareLaunchArgument('goal_x', default_value='-9.0'),
    DeclareLaunchArgument('goal_y', default_value='0.0'),
]


def _launch(pkg, name, args):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items())


def simulation(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory('dynabarn2')
    world = os.path.join(share, 'worlds', get('world_file') + '.sdf')
    server_only = '' if get('gui') == 'true' else ' -s'

    # Clearpath generates files next to robot.yaml, so give each trial its own folder.
    setup_path = tempfile.mkdtemp(prefix='dynabarn2_robot_')
    shutil.copy(os.path.join(share, 'config', 'robot.yaml'), setup_path)

    gazebo = _launch('ros_gz_sim', 'gz_sim.launch.py',
                     {'gz_args': f'{world}{server_only} -r -v 1'})

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='trial_bridge', output='log',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/robot/touched@std_msgs/msg/Bool[gz.msgs.Boolean',
                   '/model/robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': True}])

    robot = _launch('clearpath_gz', 'robot_spawn.launch.py', {
        'use_sim_time': 'true', 'setup_path': setup_path, 'world': 'default', 'rviz': 'false',
        'generate': 'true', 'x': get('x'), 'y': get('y'), 'z': '0.3', 'yaw': get('yaw')})

    return [gazebo, bridge, robot]


def navigation(context):
    planner = LaunchConfiguration('planner').perform(context)
    if planner == 'none':
        return []
    # Give the robot's bringup time to come up before Nav2 starts.
    return [TimerAction(period=15.0, actions=[
        LogInfo(msg=f'Starting Nav2 baseline: {planner}'),
        _launch('dynabarn2', 'nav2_baseline.launch.py',
                {'planner': planner,
                 'log_level': LaunchConfiguration('nav_log_level').perform(context)})])]


def generate_launch_description():
    runner = Node(
        package='dynabarn2', executable='trial_runner.py', output='screen',
        parameters=[{'world': LaunchConfiguration('world_file'),
                     'planner': LaunchConfiguration('planner'),
                     'out_file': LaunchConfiguration('out_file')},
                    {'timeout': ParameterValue(LaunchConfiguration('timeout'), value_type=float),
                     **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                        for name in ('x', 'y', 'yaw', 'goal_x', 'goal_y')}}],
        on_exit=[Shutdown(reason='trial finished')])
    return LaunchDescription(ARGUMENTS + [
        OpaqueFunction(function=simulation),
        OpaqueFunction(function=navigation),
        TimerAction(period=10.0, actions=[runner]),
    ])
