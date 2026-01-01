import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument # <--- CHANGED: Added DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    sentry_sim_dir = get_package_share_directory('sentry_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    slam_params_file = os.path.join(sentry_sim_dir, 'config', 'mapper_params_online_async.yaml')

    # Config file
    nav2_params_file = os.path.join(sentry_sim_dir,
                                   'config', 'nav2', 'nav2_sentry.yaml')
    # amcl_params_file = os.path.join(sentry_sim_dir,
    #                                  'config', 'nav2', 'nav2_amcl.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Override the global_frame for all of the nav2 nodes
    param_substitutions = {
        'use_sim_time': use_sim_time,
        # 'global_frame': LaunchConfiguration('global_frame', default='odom')
    }
    
    configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)


    # lidar_utils_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('lidar_utils'),
    #             'launch',
    #             'lidar_utils_launch.py'
    #         )
    #     )
    # )

    # odom_broadcast_node = Node(
    #     package='sentry_odom',
    #     executable='odom_frame_broadcast_node',
    #     name='odom_frame_broadcast',
    # )

    # tf_broadcast_node = Node(
    #     package='sentry_odom',
    #     executable='transform_relay_node',
    #     name='transform_relay',
    # )

    # rosco_interface_node = Node(
    #     package='rosco_interface',
    #     executable='rosco_interface',
    #     name='rosco_interface',
    #     output='screen',
    #     respawn=True,
    #     respawn_delay=2
    # )

    # Nav2 launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': configured_params,
                          'autostart': 'True'}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        # add params
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        # declare argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # lidar_utils_launch,
        # odom_broadcast_node,
        # tf_broadcast_node,
        # rosco_interface_node,
        nav_launch,
        slam_launch,
    ])