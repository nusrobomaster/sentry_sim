from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
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
                'gz_args': [
                    PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'worlds', '2023_v_4_1.world']),
                    TextSubstitution(text=' -s '), LaunchConfiguration('paused'),
                    TextSubstitution(text=' -g '), LaunchConfiguration('gui')
                ]
            }.items(),
        ),

        # Spawn the robot in Gazebo using ros_gz's spawn_entity node
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_urdf',
            output='screen',
            arguments=[
                '-file', Command(['xacro ', PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'urdf', 'base_car.urdf.xacro'])]),
                '-name', 'mobile_base',
                '-x', LaunchConfiguration('x_pos'),
                '-y', LaunchConfiguration('y_pos'),
                '-z', LaunchConfiguration('z_pos')
            ],
        ),

        # Load configuration YAML file for robot control
        # Node(
        #     package='ros2param',
        #     executable='load',
        #     name='config_loader',
        #     arguments=[PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'config', 'sc_config.yaml'])],
        # ),

        # Robot state publisher node (publishing transforms for robot's joints)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': 30.0
            }],
        ),

        # Control node for sending velocity commands
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
            prefix='xterm -e',
        ),

        # Static transform publisher for map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '1000']
        ),

        # Group for RViz (if enabled)
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'rviz', 'rm_lidar.rviz'])],
                output='screen',
                condition=IfCondition(LaunchConfiguration('is_open_rviz'))
            )
        ])
    ])
