# ~/tb3_ws/src/dynabarn_turtlebot/launch/simple_test.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Args
    model = DeclareLaunchArgument('model', default_value='burger', description='burger|waffle|waffle_pi')
    gui   = DeclareLaunchArgument('gui',   default_value='true')
    name  = DeclareLaunchArgument('name',  default_value='turtlebot3')

    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.01')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Empty world
    empty_world = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])

    # Start Gazebo (no auto-spawn)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': empty_world, 'gui': LaunchConfiguration('gui'), 'verbose': 'true'}.items()
    )

    # Path to SDF model: <turtlebot3_gazebo>/models/turtlebot3_<model>/model.sdf
    tb3_sdf = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'), 'models',
        PythonExpression(["'turtlebot3_' + '", LaunchConfiguration('model'), "'"]),
        'model.sdf'
    ])

    # Spawn from file (no /robot_description needed)
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
        ]
    )

    return LaunchDescription([
        model, gui, name, x, y, z, yaw,
        gazebo,
        spawn
    ])
# sudo apt-get update && sudo apt-get install -y ros-humble-teleop-twist-keyboard
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# export GAZEBO_AUDIO=0