from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node definition for terrainAnalysis
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize': 0.05, # m
                'decayTime': 2.0, # s
                'noDecayDis': 4.0, # m
                'clearingDis': 8.0, # m
                'useSorting': True,
                'quantileZ': 0.25,
                'considerDrop': True,
                'limitGroundLift': False, # only applicable if useSorting
                'maxGroundLift': 0.2, # only applicable if useSorting
                'clearDyObs': True, # clear dynamic obstacle spots
                'minDyObsDis': 0.3,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': -0.5,
                'absDyObsRelZThre': 0.2,
                'minDyObsVFOV': -16.0,
                'maxDyObsVFOV': 16.0,
                'minDyObsPointNum': 1,
                'noDataObstacle': True, # negative obstacle
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 10,
                'vehicleHeight': 1.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ': -2.5,
                'maxRelZ': 2.0,
                'disRatioZ': 0.2 # pointcloud processing height and distance ratio, related to lidar specs
            }]
        )
    ])
