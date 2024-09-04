from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for graph_topic
        DeclareLaunchArgument('graph_topic', default_value='/planner_nav_graph'),

        # Node definition for graph_decoder
        Node(
            package='graph_decoder',
            executable='graph_decoder',
            name='graph_decoder',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('graph_decoder'), 'config', 'default.yaml'])],
            remappings=[
                ('/planner_nav_graph', LaunchConfiguration('graph_topic')),
            ]
        ),

        # Commented out RViz node (as in the original ROS 1 launch file)
        # Uncomment and adapt if needed in the future
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='decodeRiz',
        #     arguments=['-d', PathJoinSubstitution([FindPackageShare('graph_decoder'), 'rviz', 'decoder.rviz'])],
        #     output='screen',
        # )
    ])
