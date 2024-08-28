import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    sentry_gazebo_share = get_package_share_directory('sentry_gazebo')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('paused', default_value='false', description='Start the simulation paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI'),
        DeclareLaunchArgument('verbose', default_value='true', description='Enable verbose mode'),
        # Initial position of simulated robot
        DeclareLaunchArgument('x_pos', default_value='8', description='Initial X position of the robot'),
        DeclareLaunchArgument('y_pos', default_value='6', description='Initial Y position of the robot'),
        DeclareLaunchArgument('z_pos', default_value='0.5', description='Initial Z position of the robot'),
        DeclareLaunchArgument('R_pos', default_value='0', description='Initial Roll position of the robot'),
        DeclareLaunchArgument('P_pos', default_value='0', description='Initial Pitch position of the robot'),
        DeclareLaunchArgument('Y_pos', default_value='0', description='Initial Yaw position of the robot'),
        DeclareLaunchArgument('is_open_rviz', default_value='true', description='Launch RViz'),

        # Include Gazebo empty world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'debug': 'false',
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'headless': 'false',
                'verbose': LaunchConfiguration('verbose'),
                'world': os.path.join(sentry_gazebo_share, 'worlds', '2023_v_4_1.world')
            }.items(),
        ),

        # Load the robot description from xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', os.path.join(sentry_gazebo_share, 'urdf', 'base_car.urdf.xacro')]),
                'publish_frequency': 30.0
            }],
            output='screen',
        ),

        # Spawn the URDF model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-unpause', 
                '-entity', 'mobile_base',
                '-x', LaunchConfiguration('x_pos'),
                '-y', LaunchConfiguration('y_pos'),
                '-z', LaunchConfiguration('z_pos'),
                '-param', 'robot_description'
            ],
            output='screen',
        ),

        # Load controller configurations
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                os.path.join(sentry_gazebo_share, 'config', 'sc_config.yaml')
            ],
            output='screen'
        ),

        # Sentry control key node
        Node(
            package='sentry_controller',
            executable='sentry_control_key',
            name='sentry_control_key',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/A/car0/cmd_vel',
                'velocity_linear': 3,
                'velocity_angular': 3
            }],
            prefix='xterm -e'
        ),

        # Static transform publisher (map to odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '1000']
        ),

        # Launch RViz if specified
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', os.path.join(sentry_gazebo_share, 'rviz', 'rm_lidar.rviz')],
            ),
        ], condition=LaunchConfiguration('is_open_rviz') == 'true')
    ])

