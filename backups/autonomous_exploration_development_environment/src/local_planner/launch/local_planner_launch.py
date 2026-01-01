from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('sensorOffsetX', default_value='0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0'),
        DeclareLaunchArgument('twoWayDrive', default_value='false'),
        DeclareLaunchArgument('maxSpeed', default_value='2.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='2.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('goalX', default_value='0'),
        DeclareLaunchArgument('goalY', default_value='0'),

        # localPlanner node
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[{
                'pathFolder': LaunchConfiguration('pathFolder', default='paths'),
                'vehicleLength': 0.6,
                'vehicleWidth': 0.6,
                'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
                'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
                'twoWayDrive': LaunchConfiguration('twoWayDrive'),
                'laserVoxelSize': 0.05,
                'terrainVoxelSize': 0.2,
                'useTerrainAnalysis': True,
                'checkObstacle': True,
                'checkRotObstacle': False,
                'adjacentRange': 4.25,
                'obstacleHeightThre': 0.15,
                'groundHeightThre': 0.1,
                'costHeightThre': 0.1,
                'costScore': 0.02,
                'useCost': False,
                'pointPerPathThre': 2,
                'minRelZ': -0.5,
                'maxRelZ': 0.25,
                'maxSpeed': LaunchConfiguration('maxSpeed'),
                'dirWeight': 0.02,
                'dirThre': 90.0,
                'dirToVehicle': False,
                'pathScale': 1.25,
                'minPathScale': 0.75,
                'pathScaleStep': 0.25,
                'pathScaleBySpeed': True,
                'minPathRange': 1.0,
                'pathRangeStep': 0.5,
                'pathRangeBySpeed': True,
                'pathCropByGoal': True,
                'autonomyMode': LaunchConfiguration('autonomyMode'),
                'autonomySpeed': LaunchConfiguration('autonomySpeed'),
                'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay'),
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange': 0.5,
                'goalX': LaunchConfiguration('goalX'),
                'goalY': LaunchConfiguration('goalY')
            }]
        ),

        # pathFollower node
        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[{
                'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
                'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
                'pubSkipNum': 1,
                'twoWayDrive': LaunchConfiguration('twoWayDrive'),
                'lookAheadDis': 0.5,
                'yawRateGain': 7.5,
                'stopYawRateGain': 7.5,
                'maxYawRate': 90.0,
                'maxSpeed': LaunchConfiguration('maxSpeed'),
                'maxAccel': 2.5,
                'switchTimeThre': 1.0,
                'dirDiffThre': 0.1,
                'stopDisThre': 0.2,
                'slowDwnDisThre': 0.85,
                'useInclRateToSlow': False,
                'inclRateThre': 120.0,
                'slowRate1': 0.25,
                'slowRate2': 0.5,
                'slowTime1': 2.0,
                'slowTime2': 2.0,
                'useInclToStop': False,
                'inclThre': 45.0,
                'stopTime': 5.0,
                'noRotAtStop': False,
                'noRotAtGoal': True,
                'autonomyMode': LaunchConfiguration('autonomyMode'),
                'autonomySpeed': LaunchConfiguration('autonomySpeed'),
                'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay')
            }]
        ),

        # vehicleTransPublisher static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vehicleTransPublisher',
            arguments=[LaunchConfiguration('sensorOffsetX'), LaunchConfiguration('sensorOffsetY'), '0', '0', '0', '0', 'sensor', 'vehicle', '1000']
        ),

        # sensorTransPublisher static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensorTransPublisher',
            arguments=['0', '0', LaunchConfiguration('cameraOffsetZ'), '-1.5707963', '0', '-1.5707963', 'sensor', 'camera', '1000']
        ),
    ])
