import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sentry_global_planner_share = get_package_share_directory('sentry_global_planner')

    costmap_common_params_yaml = os.path.join(sentry_global_planner_share, 'param', 'costmap_common_params.yaml')
    local_costmap_params_yaml = os.path.join(sentry_global_planner_share, 'param', 'local_costmap_params.yaml')
    global_costmap_params_yaml = os.path.join(sentry_global_planner_share, 'param', 'global_costmap_params.yaml')
    base_local_planner_params_yaml = os.path.join(sentry_global_planner_share, 'param', 'base_local_planner_params.yaml')
    global_planner_params_yaml = os.path.join(sentry_global_planner_share, 'param', 'global_planner_params.yaml')
    map_server_yaml = os.path.join(sentry_global_planner_share, 'map', 'map_server.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='move_base',
            executable='move_base',
            name='move_base1',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                costmap_common_params_yaml,
                local_costmap_params_yaml,
                global_costmap_params_yaml,
                base_local_planner_params_yaml,
                global_planner_params_yaml
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel_mobe_base'),
                ('/map', '/prior_map'),
                ('/odom', '/state_estimation')
            ]
        ),
        
        Node(
            package='map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_server_yaml],
            remappings=[
                ('/map', '/prior_map')
            ]
        )
    ])

