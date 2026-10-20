# benchmarking.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configurable delay between launches
    delay_sec_arg = DeclareLaunchArgument(
        'delay_sec', default_value='5.0',
        description='Seconds to wait after imagedetector starts before launching end_to_end'
    )
    delay_sec = LaunchConfiguration('delay_sec')

    pkg = 'dynabarn_turtlebot'

    # Include imagedetector.launch.py
    imagedetector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', 'imagedetector.launch.py'])
        ),
        # If imagedetector needs args, pass them here:
        # launch_arguments={'some_arg': 'value'}.items()
    )

    # Include end_to_end.launch.py (after delay)
    end_to_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', 'end_to_end.launch.py'])
        ),
        # If end_to_end needs args, pass them here:
        # launch_arguments={'goal_x': '10.0', 'goal_y': '0.0'}.items()
    )

    delayed_end_to_end = TimerAction(period=delay_sec, actions=[end_to_end])

    return LaunchDescription([
        delay_sec_arg,
        imagedetector,
        delayed_end_to_end,
    ])
