#!/usr/bin/env python3
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution

DEFAULT_WORLDS_DIR = "/ws/src/DynaBARN/worlds"

def _check_world_path(context):
    worlds_dir = Path(LaunchConfiguration('worlds_dir').perform(context))
    world = LaunchConfiguration('world').perform(context)
    full = worlds_dir / world
    if not full.exists():
        print(f"[bringup_dynabarn] World not found: {full}")
    return []

def generate_launch_description():
    worlds_dir_arg = DeclareLaunchArgument(
        'worlds_dir',
        default_value=DEFAULT_WORLDS_DIR,
        description='Directory containing DynaBARN world files (not a rospkg).'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world.world',
        description='World filename inside the worlds_dir.'
    )

    # Append to existing env vars correctly (no shell-style ${...}).
    set_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            LaunchConfiguration('worlds_dir'),
            TextSubstitution(text=':'),
            EnvironmentVariable(name='GAZEBO_RESOURCE_PATH', default_value='')
        ]
    )

    set_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            LaunchConfiguration('worlds_dir'),
            TextSubstitution(text=':'),
            EnvironmentVariable(name='GAZEBO_MODEL_PATH', default_value='')
        ]
    )

    # Optional: silence audio device warnings in containers
    set_audio_off = SetEnvironmentVariable(name='GAZEBO_AUDIO', value='0')

    gzserver = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            [LaunchConfiguration('worlds_dir'), '/', LaunchConfiguration('world')]
        ],
        cwd=LaunchConfiguration('worlds_dir'),
        output='screen'
    )

    # gzclient = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen'
    # )

    return LaunchDescription([
        worlds_dir_arg,
        world_arg,
        set_gazebo_resource,
        set_gazebo_model,
        set_audio_off,
        OpaqueFunction(function=_check_world_path),
        gzserver,
        # gzclient
    ])
