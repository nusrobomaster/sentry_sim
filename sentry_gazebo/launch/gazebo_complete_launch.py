import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_project = get_package_share_directory('sentry_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file_path = '/home/dinvsh/sentry_sim/sentry_gazebo/map/rmul_map.yaml'
    
    nav2_params_file = os.path.join(pkg_project, 'config', 'nav2', 'nav2_sentry.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file_path 
    }
    
    configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    argument_declarations = [
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('x_pos', default_value='8'),
        DeclareLaunchArgument('y_pos', default_value='6'),
        DeclareLaunchArgument('z_pos', default_value='0.5'),
        DeclareLaunchArgument('R_pos', default_value='0'),
        DeclareLaunchArgument('P_pos', default_value='0'),
        DeclareLaunchArgument('Y_pos', default_value='0'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    xacro_file = os.path.join(pkg_project, 'urdf/', 'test_robot.xacro')
    assert os.path.exists(xacro_file), "The test_robot.xacro doesnt exist in " + str(xacro_file)
    robot_desc = xacro.process_file(xacro_file).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-v 4 -r ', PathJoinSubstitution([pkg_project, 'worlds', '2023_v_4_1.sdf'])],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_urdf',
        arguments=['-topic', '/robot_description', '-name', 'testbot', '-x', '8', '-y', '6', '-z', '0.5'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc, 'publish_frequency': 30.0}],
    )

    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/lidar_points/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/default/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
        ],
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': configured_params, 'autostart': 'True'}.items()
    )

    # localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
    #     launch_arguments={
    #         'map': map_file_path, 
    #         'use_sim_time': use_sim_time, 
    #         'params_file': configured_params, 
    #         'autostart': 'True'
    #     }.items()
    # )

    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom'],
        output='screen'
    )

    return LaunchDescription(
        argument_declarations + [
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        nav_launch,          
        # localization_launch,
        tf_map_to_odom
    ])