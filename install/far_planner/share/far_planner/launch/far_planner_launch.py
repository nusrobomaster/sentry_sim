from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation'),
        DeclareLaunchArgument('terrain_cloud_topic', default_value='/terrain_map_ext'),
        DeclareLaunchArgument('terrain_local_topic', default_value='/terrain_map'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/registered_scan'),
        DeclareLaunchArgument('config', default_value='default'),

        # Node definition for far_planner
        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('far_planner'), 'config', LaunchConfiguration('config') + '.yaml'])],
            remappings=[
                ('/odom_world', LaunchConfiguration('odom_topic')),
                ('/terrain_cloud', LaunchConfiguration('terrain_cloud_topic')),
                ('/scan_cloud', LaunchConfiguration('scan_cloud_topic')),
                ('/terrain_local_cloud', LaunchConfiguration('terrain_local_topic'))
            ]
        ),

        # Node definition for RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='far_rviz',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('far_planner'), 'rviz', LaunchConfiguration('config') + '.rviz'])],
            respawn=True,
            output='screen'
        ),

        # Launch Graph Manager for saving and loading graph
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('graph_decoder'), 'launch', 'decoder_launch.py'])])
        )
    ])
