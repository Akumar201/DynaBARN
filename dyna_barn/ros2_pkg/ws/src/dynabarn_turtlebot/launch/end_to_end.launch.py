from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Reuse your existing bringup (gazebo + world + spawn)
    gui   = DeclareLaunchArgument('gui',   default_value='true')
    name  = DeclareLaunchArgument('name',  default_value='turtlebot3')
    x     = DeclareLaunchArgument('x',     default_value='-8.0')
    y     = DeclareLaunchArgument('y',     default_value='0.0')
    z     = DeclareLaunchArgument('z',     default_value='0.01')
    yaw   = DeclareLaunchArgument('yaw',   default_value='0.0')

    worlds_dir = DeclareLaunchArgument(
        'worlds_dir', default_value='/ws/src/DynaBARN/worlds'
    )
    world = DeclareLaunchArgument(
        'world', default_value='world.world'
    )

    world_path = [LaunchConfiguration('worlds_dir'),
                  TextSubstitution(text='/'),
                  LaunchConfiguration('world')]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={
            'world': world_path,
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true'
        }.items()
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

    # Controller node parameters (goal in odom frame)
    goal_x = DeclareLaunchArgument('goal_x', default_value='10.0')
    goal_y = DeclareLaunchArgument('goal_y', default_value='0.0')

    controller = Node(
        package='dynabarn_turtlebot',
        executable='end_to_end',
        name='end_to_end',
        output='screen',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            # Optional tuning:
            # 'xy_tolerance': 0.1,
            # 'yaw_tolerance': 0.2,
            # 'lin_kp': 0.8,
            # 'ang_kp': 2.0,
            # 'max_linear_speed': 0.25,
            # 'max_angular_speed': 1.2,
            # 'obstacle_stop_dist': 0.35,
        }],
    )

    # Image viewer subscribed to annotated output
    image_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_annotated',
        output='screen',
        arguments=['/imagedetector/image_annotated'],
    )

    return LaunchDescription([
        gui, name, x, y, z, yaw,
        worlds_dir, world,
        gazebo,
        spawn,
        goal_x, goal_y,
        controller,
        image_viewer,
    ])
