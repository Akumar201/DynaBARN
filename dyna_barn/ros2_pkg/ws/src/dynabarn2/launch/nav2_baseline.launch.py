"""Nav2 baseline for DynaBARN trials, built on Clearpath's J100 configuration.

Mapless: the planner and both costmaps work in the odom frame from the lidar alone, as the
challenge gives no map. Only what differs from Clearpath's nav2.yaml is set here.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCAN_TOPIC = '/sensors/lidar2d_0/scan'
CMD_VEL_TOPIC = '/platform/cmd_vel'
ODOM_TOPIC = '/platform/odom/filtered'
MAX_SPEED = 1.0                # m/s, for both baselines
J100_FOOTPRINT = '[ [0.26, 0.22], [0.26, -0.22], [-0.26, -0.22], [-0.26, 0.22] ]'  # with fenders
SENSOR_RANGE = 4.5             # m, how far obstacles are marked in the costmaps

DWB_CONTROLLER = {
    'plugin': 'dwb_core::DWBLocalPlanner',
    'debug_trajectory_details': False,
    'min_vel_x': 0.0, 'min_vel_y': 0.0, 'max_vel_x': MAX_SPEED, 'max_vel_y': 0.0,
    'max_vel_theta': 1.9, 'min_speed_xy': 0.0, 'max_speed_xy': MAX_SPEED, 'min_speed_theta': 0.0,
    'acc_lim_x': 2.5, 'acc_lim_y': 0.0, 'acc_lim_theta': 3.2,
    'decel_lim_x': -2.5, 'decel_lim_y': 0.0, 'decel_lim_theta': -3.2,
    'vx_samples': 20, 'vy_samples': 1, 'vtheta_samples': 30,
    'sim_time': 1.7, 'linear_granularity': 0.05, 'angular_granularity': 0.025,
    'transform_tolerance': 0.2, 'xy_goal_tolerance': 0.25, 'trans_stopped_velocity': 0.25,
    'short_circuit_trajectory_evaluation': True, 'stateful': True,
    'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign',
                'PathDist', 'GoalDist'],
    'BaseObstacle.scale': 0.02,
    'PathAlign.scale': 32.0, 'PathAlign.forward_point_distance': 0.1,
    'GoalAlign.scale': 24.0, 'GoalAlign.forward_point_distance': 0.1,
    'PathDist.scale': 32.0, 'GoalDist.scale': 24.0,
    'RotateToGoal.scale': 32.0, 'RotateToGoal.slowing_factor': 5.0,
    'RotateToGoal.lookahead_time': -1.0,
}


def build_params(planner):
    """Clearpath's J100 Nav2 parameters, turned into a mapless baseline."""
    source = os.path.join(get_package_share_directory('clearpath_nav2_demos'),
                          'config', 'j100', 'nav2.yaml')
    with open(source) as f:
        params = yaml.safe_load(f)

    params['bt_navigator']['ros__parameters']['global_frame'] = 'odom'
    params['behavior_server']['ros__parameters']['global_frame'] = 'odom'
    params['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = CMD_VEL_TOPIC
    params['velocity_smoother']['ros__parameters']['max_velocity'] = [MAX_SPEED, 0.0, 1.9]
    params['velocity_smoother']['ros__parameters']['min_velocity'] = [-0.35, 0.0, -1.9]

    local = params['local_costmap']['local_costmap']['ros__parameters']
    glob = params['global_costmap']['global_costmap']['ros__parameters']
    glob['global_frame'] = 'odom'
    local['width'] = local['height'] = 9
    glob['width'] = glob['height'] = 44  # room for the whole 20 m route plus goal
    for costmap, sensor_layer in ((local, 'voxel_layer'), (glob, 'obstacle_layer')):
        costmap['footprint'] = J100_FOOTPRINT
        costmap['plugins'] = [sensor_layer, 'inflation_layer']
        costmap.pop('static_layer')
        costmap['inflation_layer']['inflation_radius'] = 0.6
        scan = costmap[sensor_layer]['scan']
        scan['topic'] = SCAN_TOPIC
        scan['obstacle_max_range'] = SENSOR_RANGE
        scan['raytrace_max_range'] = SENSOR_RANGE + 1.0

    controller = params['controller_server']['ros__parameters']
    # Without this the controller reads speed 0 forever: MPPI then can't exceed one acceleration
    # step per cycle (about 0.2 m/s), and DWB samples around the wrong speed.
    controller['odom_topic'] = ODOM_TOPIC
    controller['general_goal_checker']['stateful'] = True
    controller['failure_tolerance'] = 2.0
    controller['progress_checker']['movement_time_allowance'] = 30.0
    if planner == 'dwb':
        controller['FollowPath'] = dict(DWB_CONTROLLER)
    else:
        mppi = controller['FollowPath']
        mppi['vx_max'] = MAX_SPEED
        mppi['vy_max'] = 0.0
        mppi['visualize'] = False
        # Clearpath's values suit 0.5 m/s. The path handed to MPPI must outreach its horizon
        # (time_steps * model_dt * vx_max), and samples must spread enough to reach full speed.
        mppi['prune_distance'] = 1.2 * mppi['time_steps'] * mppi['model_dt'] * MAX_SPEED
        mppi['vx_std'] = 0.4
    return params


def launch_setup(context):
    planner = LaunchConfiguration('planner').perform(context)
    with tempfile.NamedTemporaryFile('w', prefix=f'dynabarn2_nav2_{planner}_', suffix='.yaml',
                                     delete=False) as f:
        yaml.safe_dump(build_params(planner), f)
    launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                               'navigation_launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'use_sim_time': 'true', 'params_file': f.name,
                          'use_composition': 'False', 'autostart': 'true',
                          'log_level': LaunchConfiguration('log_level').perform(context)}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='mppi', choices=['mppi', 'dwb']),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=launch_setup),
    ])
