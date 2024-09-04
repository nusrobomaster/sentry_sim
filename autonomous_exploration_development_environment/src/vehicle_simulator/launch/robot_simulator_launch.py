from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare all the launch arguments
        DeclareLaunchArgument('sensorOffsetX', default_value='0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0'),

        DeclareLaunchArgument('vehicleX', default_value='0'),
        DeclareLaunchArgument('vehicleY', default_value='0'),
        DeclareLaunchArgument('vehicleZ', default_value='0'),
        DeclareLaunchArgument('terrainZ', default_value='0'),
        DeclareLaunchArgument('vehicleYaw', default_value='0'),

        DeclareLaunchArgument('terrainVoxelSize', default_value='0.05'),
        DeclareLaunchArgument('groundHeightThre', default_value='0.1'),
        DeclareLaunchArgument('adjustZ', default_value='true'),
        DeclareLaunchArgument('terrainRadiusZ', default_value='1.0'),
        DeclareLaunchArgument('minTerrainPointNumZ', default_value='5'),
        DeclareLaunchArgument('smoothRateZ', default_value='0.5'),
        DeclareLaunchArgument('adjustIncl', default_value='true'),
        DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0'),
        DeclareLaunchArgument('minTerrainPointNumIncl', default_value='200'),
        DeclareLaunchArgument('smoothRateIncl', default_value='0.5'),
        DeclareLaunchArgument('InclFittingThre', default_value='0.2'),
        DeclareLaunchArgument('maxIncl', default_value='30.0'),

        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='garage'),

        # Node definition
        Node(
            package='vehicle_simulator',
            executable='robotSimulator',
            name='vehicleSimulator',
            output='screen',
            parameters=[{
                'use_gazebo_time': False,
                'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
                'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
                'vehicleHeight': LaunchConfiguration('vehicleHeight'),
                'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
                'vehicleX': LaunchConfiguration('vehicleX'),
                'vehicleY': LaunchConfiguration('vehicleY'),
                'vehicleZ': LaunchConfiguration('vehicleZ'),
                'terrainZ': LaunchConfiguration('terrainZ'),
                'vehicleYaw': LaunchConfiguration('vehicleYaw'),
                'terrainVoxelSize': LaunchConfiguration('terrainVoxelSize'),
                'groundHeightThre': LaunchConfiguration('groundHeightThre'),
                'adjustZ': LaunchConfiguration('adjustZ'),
                'terrainRadiusZ': LaunchConfiguration('terrainRadiusZ'),
                'minTerrainPointNumZ': LaunchConfiguration('minTerrainPointNumZ'),
                'smoothRateZ': LaunchConfiguration('smoothRateZ'),
                'adjustIncl': LaunchConfiguration('adjustIncl'),
                'terrainRadiusIncl': LaunchConfiguration('terrainRadiusIncl'),
                'minTerrainPointNumIncl': LaunchConfiguration('minTerrainPointNumIncl'),
                'smoothRateIncl': LaunchConfiguration('smoothRateIncl'),
                'InclFittingThre': LaunchConfiguration('InclFittingThre'),
                'maxIncl': LaunchConfiguration('maxIncl'),
            }]
        ),
    ])
