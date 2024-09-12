import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    sentry_gazebo_share = get_package_share_directory('sentry_gazebo')
    xacro_file = os.path.join(sentry_gazebo_share, 'urdf/', 'test_robot.xacro')
    assert os.path.exists(xacro_file), "The test_robot.xacro doesnt exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
 
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        # Initial position of simulated robot
        DeclareLaunchArgument('x_pos', default_value='8'),
        DeclareLaunchArgument('y_pos', default_value='6'),
        DeclareLaunchArgument('z_pos', default_value='0.5'),
        DeclareLaunchArgument('R_pos', default_value='0'),
        DeclareLaunchArgument('P_pos', default_value='0'),
        DeclareLaunchArgument('Y_pos', default_value='0'),
        DeclareLaunchArgument('is_open_rviz', default_value='true'),

        # Include Gazebo world (using ros_gz_sim for Gazebo Fortress)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
            launch_arguments={
                'gz_args': ['-v 4 -r ', PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'worlds', '2023_v_4_1.sdf'])],
                'on_exit_shutdown': 'true'
            }.items(),
        ),

        # Spawn the robot in Gazebo using ros_gz's spawn_entity node
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_urdf',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'testbot',
                '-x', '8',
                '-y', '6',
                '-z', '0.5'
            ],
            output='screen'
        ),

        # # Load configuration YAML file for robot control
        # # Node(
        # #     package='ros2param',
        # #     executable='load',
        # #     name='config_loader',
        # #     arguments=[PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'config', 'sc_config.yaml'])],
        # # ),

        # # Robot state publisher node (publishing transforms for robot's joints)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_desc,
                'publish_frequency': 30.0
            }],
        ),

        Node(
            package='teleop_twist_keyboard',
            name='teleop',
            executable="teleop_twist_keyboard",
            remappings=[
                ('/cmd_vel', '/A/car0/cmd_vel'),
                ],
            output='screen',
            prefix = 'xterm -e',
            
        ),

        Node(
            package='ros_gz_bridge',
            name='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/A/car0/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            output='screen'
        )

        # # Static transform publisher for map to odom
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map2odom',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '1000']
        # ),

        # Group for RViz (if enabled)
        # GroupAction([
        #     Node(
        #         package='rviz2',
        #         executable='rviz2',
        #         name='rviz',
        #         arguments=['-d', PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'rviz', 'rm_lidar.rviz'])],
        #         output='screen',
        #         condition=IfCondition(LaunchConfiguration('is_open_rviz'))
        #     )
        # ])
    ])
