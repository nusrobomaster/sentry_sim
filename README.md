# sentry_sim

Sentry simulation on gazebo. Modified form the original forked library to make it compatiable for ROS 2 Humble with ignition gazebo fortress.

## Progress
* Able to launch the RMUC arena in Gazebo Fortress
* Robot is spawned from a Xacro/URDF model with correct physics

Robot can be moved:
* Manually using keyboard teleoperation
* Autonomously using RViz2 2D Goal Pose

Latest testing is done using:
* Launch file: gazebo_rmuc_test_launch.py
* Robot description: test_robot.xacro

## Build
```SHELL
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build only the simulation package
colcon build --packages-select sentry_gazebo --symlink-install
source install/setup.bash
```

## Launch Simulation
Start Gazebo Fortress and spawn the robot:
```SHELL
ros2 launch sentry_gazebo gazebo_rmuc_test_launch.py
```

## SLAM and Navigation
Start SLAM Toolbox and navigation nodes:
```SHELL
ros2 launch sentry_gazebo mapping_nav_launch.py use_sim_time:=true
```

## Visualisation (RViz2)
Run RViz2 with simulation time enabled:
```SHELL
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
Fixed Frame: Set to map. Displays: Add Grid, TF, RobotModel, LaserScan (Topic: /scan), Map (Topic: /map), and Path (Topic: /plan).
* Use 2D Goal Pose to move the robot autonomously

## Manual Control
Optional keyboard teleoperation:
```SHELL
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Dependencies
Install Gazebo Fortress, Navigation2, and SLAM tools
```SHELL
sudo apt-get install ros-humble-ros-gz \
                     ros-humble-ros-gz-bridge \
                     ros-humble-slam-toolbox \
                     ros-humble-navigation2 \
                     ros-humble-nav2-bringup \
                     ros-humble-teleop-twist-keyboard
```

