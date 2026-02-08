# sentry_sim

Sentry simulation on Gazebo Fortress with ROS 2 Humble. Features skid-steer kinematics, autonomous navigation, and 3D LiDAR-inertial odometry.

## Features

**Simulation & Physics**
* 4-wheel skid-steer drive with differential control
* 3D LiDAR sensor (Livox Mid-360 simulation via `gpu_lidar`)
* IMU sensor integration

**Localization & Mapping**
* FAST-LIO for real-time LiDAR-inertial odometry and mapping
* Navigation2 with AMCL localization
* SLAM Toolbox support

**Autonomous Navigation**
* Finite State Machine (FSM) for autonomous behavior
* Interrupt logic for enemy detection and low-HP triggers
* Remote command interface for manual control

## Dependencies

### Core ROS 2 Packages
```bash
sudo apt install ros-humble-ros-gz \
                 ros-humble-ros-gz-bridge \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-teleop-twist-keyboard
```

### FAST-LIO
Included in this repo under `./FAST_LIO/`. See [upstream repository](https://github.com/hku-mars/FAST_LIO) for details.

**Note:** Code has been modified to work with simulated PointCloud2 data.

### Livox Drivers
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## Build

```bash
cd ~/sentry_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Usage

**Note:** Source your workspace in every new terminal:
```bash
cd ~/sentry_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. Simulation Only

```bash
ros2 launch sentry_gazebo gazebo_complete_launch.py
```

Launches Gazebo with robot, sensors, and ros_gz_bridge.

### 2. Mapping with FAST-LIO

```bash
# Terminal 1: Simulation
ros2 launch sentry_gazebo gazebo_complete_launch.py

# Terminal 2: FAST-LIO
ros2 launch fast_lio mapping.launch.py

# Terminal 3: Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive around to build the map.

### 3. Navigation with Nav2

```bash
# Terminal 1: Simulation
ros2 launch sentry_gazebo gazebo_complete_launch.py

# Terminal 2: Navigation stack
ros2 launch sentry_nav mapping_nav_launch.py

# Terminal 3: RViz
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**RViz Setup:**
- Fixed Frame: `map`
- Add Map: `/map`
- Add LaserScan: `/scan`
- Add Path: `/plan`

Use **2D Pose Estimate** to set initial pose, then **2D Goal Pose** for navigation.

### 4. Autonomous Behavior (FSM)

```bash
# Terminal 1: Simulation
ros2 launch sentry_gazebo gazebo_complete_launch.py

# Terminal 2: Nav2
ros2 launch sentry_nav mapping_nav_launch.py

# Terminal 3: RViz
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

# Terminal 4: FSM
python3 src/sentry_behavior/scripts/test_fsm.py

# Terminal 5: Remote control
python3 src/sentry_behavior/scripts/remote.py
```

**Remote Commands:**
- `1`: Start Match (Navigate to Central Zone)
- `2`: Retreat (Navigate to Supply Zone)  
- `3`: Enemy Detected (Interrupt & aim)
- `4`: Clear Enemy (Resume task)

**Note:** FSM takes control of navigation - disable it for manual driving.
