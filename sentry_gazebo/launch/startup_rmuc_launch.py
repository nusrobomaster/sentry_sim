import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sentry_gazebo_share = get_package_share_directory('sentry_gazebo')
    vehicle_simulator_share = get_package_share_directory('vehicle_simulator')
    far_planner_share = get_package_share_directory('far_planner')

    return LaunchDescription([
        # Simulation parameters
        DeclareLaunchArgument('paused', default_value='false', description='Start the simulation paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI'),
        # Initial position of simulated robot
        DeclareLaunchArgument('x_pos', default_value='8', description='Initial X position of the robot'),
        DeclareLaunchArgument('y_pos', default_value='6', description='Initial Y position of the robot'),
        DeclareLaunchArgument('z_pos', default_value='0.1', description='Initial Z position of the robot'),
        DeclareLaunchArgument('R_pos', default_value='0', description='Initial Roll position of the robot'),
        DeclareLaunchArgument('P_pos', default_value='0', description='Initial Pitch position of the robot'),
        DeclareLaunchArgument('Y_pos', default_value='0', description='Initial Yaw position of the robot'),
        # Autonomous Exploration Development Environment parameters
        DeclareLaunchArgument('world_name', default_value='2023_v_4_1', description='World name'),
        DeclareLaunchArgument('checkTerrainConn', default_value='true', description='Check Terrain Connectivity'),
        DeclareLaunchArgument('maxSpeed', default_value='2', description='Maximum Speed'),
        DeclareLaunchArgument('autonomySpeed', default_value='2', description='Autonomy Speed'),

        # Launch gazebo environment, load map and robot, and add controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sentry_gazebo_share, 'launch', 'gazebo_rmuc_launch.py')
            ),
            launch_arguments={
                'x_pos': LaunchConfiguration('x_pos'),
                'y_pos': LaunchConfiguration('y_pos'),
                'z_pos': LaunchConfiguration('z_pos'),
                'R_pos': LaunchConfiguration('R_pos'),
                'P_pos': LaunchConfiguration('P_pos'),
                'Y_pos': LaunchConfiguration('Y_pos'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'paused': LaunchConfiguration('paused'),
                'gui': LaunchConfiguration('gui'),
                'is_open_rviz': 'false',
            }.items()
        ),

        # Launch Autonomous Exploration Development Environment, load map analysis, partial navigation
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(vehicle_simulator_share, 'launch', 'system_rmuc_launch.py')
        #     ),
        #     launch_arguments={
        #         'world_name': LaunchConfiguration('world_name'),
        #         'vehicleX': LaunchConfiguration('x_pos'),
        #         'vehicleY': LaunchConfiguration('y_pos'),
        #         'gazebo_gui': LaunchConfiguration('gui'),
        #         'checkTerrainConn': LaunchConfiguration('checkTerrainConn'),
        #         'maxSpeed': LaunchConfiguration('maxSpeed'),
        #         'autonomySpeed': LaunchConfiguration('autonomySpeed'),
        #     }.items()
        # ),

        # # Launch far planner
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(far_planner_share, 'launch', 'far_planner_launch.py')
        #     )
        # ),
    ])
