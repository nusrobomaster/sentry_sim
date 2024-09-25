# Description: Launch file for Gazebo simulation of the RMUC test track
# Author: 
# Created: 
# Template: https://gazebosim.org/docs/fortress/ros_gz_project_template_guide/

import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    # Declare launch arguments
    argument_declarations = [
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
        # DeclareLaunchArgument('is_open_rviz', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true',
                                description='Open RViz.'),
    ]

    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project = get_package_share_directory('sentry_gazebo') # 寻找sentry_gazebo包的路径，返回的是一个字符串路径
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Load the xacro file from "description" package
    xacro_file = os.path.join(pkg_project, 'urdf/', 'test_robot.xacro') # 拼接路径
    assert os.path.exists(xacro_file), "The test_robot.xacro doesnt exist in " + str(xacro_file) # 断言，如果路径不存在，报错

    robot_description_config = xacro.process_file(xacro_file) # 读取xacro文件
    robot_desc = robot_description_config.toxml() # 转换为xml格式

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),  # 包含启动描述文件
        launch_arguments={
            'gz_args': ['-v 4 -r ', PathJoinSubstitution([pkg_project, 'worlds', '2023_v_4_1.sdf'])],  # 导入世界文件，日志等级为4，以实际时间运行
            'on_exit_shutdown': 'true'  # 退出时关闭       
        }.items()  # 转换为字典项
    )

    # Spawn the robot in Gazebo using ros_gz's spawn_entity node
    # Not sure whether this is necessary
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_urdf',
        arguments=[
            '-topic', '/robot_description', # 发布机器人描述，这个话题是在gz_sim.launch.py中定义的
            '-name', 'testbot',
            '-x', '8',
            '-y', '6',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Load configuration YAML file for robot control
    # Node(
    #     package='ros2param',
    #     executable='load',
    #     name='config_loader',
    #     arguments=[PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'config', 'sc_config.yaml'])],
    # ),

    # Robot state publisher node (publishing transforms for robot's joints)
    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc, # 机器人描述
            'publish_frequency': 30.0
        }],
    )

    # teleop_twist_keyboard seems no use
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        name='teleop',
        executable="teleop_twist_keyboard",
        remappings=[
            ('/cmd_vel', '/A/car0/cmd_vel'),
            ],
        output='screen',
        prefix = 'xterm -e',
        
    )

    # Group for RViz (if enabled)
    # rviz = GroupAction([
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz',
    #         arguments=['-d', PathJoinSubstitution([FindPackageShare('sentry_gazebo'), 'rviz', 'rm_lidar.rviz'])],
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('is_open_rviz'))
    #     )
    # ])

    # 连接ros和ignition，使得ros可以控制ignition
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        name='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist', # by running "ros2 topic echo /cmd_vel" can display,equals to transfer /keyboard/keypress to /cmd_vel
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU', # ros2 topic echo /imu cannot,seems bridge failed, equals to transfer /world/default.../imu to /imu
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan' # ros2 topic echo /lidar cannot
        ],
        output='screen'
    )

    # # Static transform publisher for map to odom
    # Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map2odom',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '1000']
    # ),
    return LaunchDescription(
        argument_declarations + [
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        # rviz,
        bridge,
    ])
