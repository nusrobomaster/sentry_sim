from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='2023_v_4_1'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0'),
        DeclareLaunchArgument('vehicleX', default_value='8'),
        DeclareLaunchArgument('vehicleY', default_value='6'),
        DeclareLaunchArgument('terrainZ', default_value='0.0'),
        DeclareLaunchArgument('vehicleYaw', default_value='0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('checkTerrainConn', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='2.0'),
        DeclareLaunchArgument('autonomySpeed', default_value='2.0'),

        # Launch local planner and broadcast initial target, same as the initial target at the bottom map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('local_planner'), 'launch', 'local_planner_launch.py'])]),
            launch_arguments={
                'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
                'goalX': LaunchConfiguration('vehicleX'),
                'goalY': LaunchConfiguration('vehicleY'),
                'maxSpeed': LaunchConfiguration('maxSpeed'),
                'autonomySpeed': LaunchConfiguration('autonomySpeed'),
            }.items(),
        ),

        # Launch terrain analysis mainly for obstacle avoiding
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('terrain_analysis'), 'launch', 'terrain_analysis_launch.py'])]),
        ),

        # Launch terrain analysis extension, larger analysis area, lower frequency, mainly used for high quality path planning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext_launch.py'])]),
            launch_arguments={
                'checkTerrainConn': LaunchConfiguration('checkTerrainConn'),
            }.items(),
        ),

        # Launch gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('vehicle_simulator'), 'launch', 'robot_simulator_launch.py'])]),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'vehicleHeight': LaunchConfiguration('vehicleHeight'),
                'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
                'vehicleX': LaunchConfiguration('vehicleX'),
                'vehicleY': LaunchConfiguration('vehicleY'),
                'terrainZ': LaunchConfiguration('terrainZ'),
                'vehicleYaw': LaunchConfiguration('vehicleYaw'),
                'gui': LaunchConfiguration('gazebo_gui'),
            }.items(),
        ),

        # tf transform pointcloud
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('sensor_scan_generation'), 'launch', 'sensor_scan_generation_launch.py'])]),
        ),

        # Visualization_tools
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('visualization_tools'), 'launch', 'visualization_tools_launch.py'])]),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
            }.items(),
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizGA',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz'])],
            output='screen',
            respawn=True,
        ),
    ])
