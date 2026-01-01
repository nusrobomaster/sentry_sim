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
        Node(
            package='global_planner',
            executable='planner',
            name='planner',
            output='screen',
            parameters=[
                {'planner/old_navfn_behavior': False},
                {'planner/use_quadratic': True},
                {'planner/use_dijkstra': True},
                {'planner/use_grid_path': False},
                {'planner/allow_unknown': True},
                {'planner/planner_window_x': 0.0},
                {'planner/planner_window_y': 0.0},
                {'planner/default_tolerance': 0.0},
                {'planner/publish_scale': 100},
                {'costmap/global_frame': 'map'},
                {'costmap/robot_base_frame': 'base_link'},
                {'costmap/origin_x': 0.0},
                {'costmap/origin_y': 0.0},
                {'costmap/resolution': 0.05},
                {'costmap/transform_tolerance': 0.2},
                {'costmap/update_frequency': 5.0},
                {'costmap/publish_frequency': 0.0},
                {'costmap/rolling_window': False},
                {'costmap/track_unknown_space': False},
                {'costmap/always_send_full_costmap': False},
                {'costmap/footprint_padding': 0.0},
                {'costmap/robot_radius': 0.5}
            ],
            remappings=[
                ('/planner/goal', '/goal'),
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

