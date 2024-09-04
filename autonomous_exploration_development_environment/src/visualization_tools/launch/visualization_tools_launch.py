from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for world_name
        DeclareLaunchArgument('world_name', default_value='garage'),

        # Node definition for visualizationTools
        Node(
            package='visualization_tools',
            executable='visualizationTools',
            name='visualizationTools',
            output='screen',
            parameters=[{
                'metricFile': PathJoinSubstitution([FindPackageShare('vehicle_simulator'), 'log', 'metrics']),
                'trajFile': PathJoinSubstitution([FindPackageShare('vehicle_simulator'), 'log', 'trajectory']),
                'mapFile': PathJoinSubstitution([FindPackageShare('vehicle_simulator'), 'mesh', LaunchConfiguration('world_name'), 'preview', 'pointcloud.ply']),
                'overallMapVoxelSize': 0.5,
                'exploredAreaVoxelSize': 0.3,
                'exploredVolumeVoxelSize': 0.5,
                'transInterval': 0.2,
                'yawInterval': 10.0,
                'overallMapDisplayInterval': 2,
                'exploredAreaDisplayInterval': 1
            }]
        ),

        # Node definition for realTimePlot.py
        Node(
            package='visualization_tools',
            executable='realTimePlot.py',
            name='realTimePlot',
            output='screen',
            respawn=True
        ),
    ])
