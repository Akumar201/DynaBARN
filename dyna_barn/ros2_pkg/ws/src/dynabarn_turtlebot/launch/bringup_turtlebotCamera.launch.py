# ~/tb3_ws/src/dynabarn_turtlebot/launch/simple_test.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gui   = DeclareLaunchArgument('gui',   default_value='true')
    name  = DeclareLaunchArgument('name',  default_value='turtlebot3')
    x = DeclareLaunchArgument('x',   default_value='0.0')
    y = DeclareLaunchArgument('y',   default_value='0.0')
    z = DeclareLaunchArgument('z',   default_value='0.01')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Gazebo Classic empty world
    empty_world = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': empty_world, 'gui': LaunchConfiguration('gui'), 'verbose': 'true'}.items()
    )

    # Your custom SDF with the camera added
    tb3_sdf = PathJoinSubstitution([
        FindPackageShare('dynabarn_turtlebot'), 'models', 'tb3_burger_cam', 'model.sdf'
    ])

    # Spawn directly from file (no /robot_description, no xacro, no RSP)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tb3',
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('name'),
            '-file',   tb3_sdf,
            '-x',      LaunchConfiguration('x'),
            '-y',      LaunchConfiguration('y'),
            '-z',      LaunchConfiguration('z'),
            '-Y',      LaunchConfiguration('yaw'),
        ],
    )

    return LaunchDescription([gui, name, x, y, z, yaw, gazebo, spawn])
