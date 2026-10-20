from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynabarn_turtlebot',
            executable='imagedetector',
            name='imagedetector',
            output='screen',
            parameters=[{
                'input_image_topic': '/turtlebot3/rgb_cam/image_raw',
                'output_image_topic': '/imagedetector/image_annotated',
                'min_area_px': 1200,
            }]
        )
    ])
