# ~/tb3_ws/src/dynabarn_turtlebot/launch/wander.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gui   = DeclareLaunchArgument('gui',   default_value='true')
    name  = DeclareLaunchArgument('name',  default_value='turtlebot3')
    x     = DeclareLaunchArgument('x',     default_value='0.0')
    y     = DeclareLaunchArgument('y',     default_value='0.0')
    z     = DeclareLaunchArgument('z',     default_value='0.01')
    yaw   = DeclareLaunchArgument('yaw',   default_value='0.0')

    worlds_dir = DeclareLaunchArgument(
        'worlds_dir', default_value='/ws/src/DynaBARN/worlds'
    )
    world = DeclareLaunchArgument(
        'world', default_value='world_18.world'
    )

    # Build the full path to the DynaBARN world (no env var munging)
    world_path = [LaunchConfiguration('worlds_dir'),
                  TextSubstitution(text='/'),
                  LaunchConfiguration('world')]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_path,
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'true'}.items()
    )

    tb3_sdf = [FindPackageShare('dynabarn_turtlebot'),
               TextSubstitution(text='/models/tb3_burger_cam/model.sdf')]

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

    return LaunchDescription([
        gui, name, x, y, z, yaw,
        worlds_dir, world,
        gazebo,
        spawn
    ])
