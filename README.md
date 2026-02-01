# sentry_sim

Sentry simulation on gazebo. Modified from the original forked library to make it compatible for ROS 2 Humble with Ignition Gazebo Fortress.

## Integrations & Features

**Simulation & Physics**
* Integrated skid-steer kinematics for 4-wheel drive.
* Implemented Z-offset spawning to prevent collision mesh clipping.

**Autonomous Navigation**
* Integrated Navigation2 (Nav2) with AMCL localization.
* Configured high-frequency map updates and increased odometry noise parameters to handle skid-steer slip.

**Logic & Control**
* **Finite State Machine (FSM):** Integrated test_fsm into the simulation.
* **Interrupt Logic:** Navigation tasks are now interruptible by enemy detection or low-HP triggers.
* **Remote Interface:** Added a command node for manual state triggers (Start, Retreat, Engage).

## Build

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the package
colcon build --symlink-install
source install/setup.bash
```

## Usage
**Note:** Remember to source both the ROS 2 environment and your workspace in every new terminal.
```bash
cd sentry_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. Visualisation
Launches RViz with simulation time enabled.
```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
Fixed Frame: Set to map. 
Displays: Add Grid, TF, LaserScan (Topic: /scan), Map (Topic: /map), Map (again) (Topic: /global_costmap/costmap) and Path (Topic: /plan).
* Note: If you want to move the robot autonomously (using 2D Goal Post) or manually (using teleop_twist_keyboard), turn off the FSM first.

### 2. Simulation & Navigation
Launches Gazebo Fortress, spawns the robot, and initializes Nav2/AMCL.
```bash
ros2 launch sentry_gazebo gazebo_complete_launch.py
```
Use 2D Pose Estimate in RViz to align the robot with where it's meant to be positioned on the map.

### 3. Finite State Machine
Starts the autonomous decision-making node.
```bash
python3 sentry_gazebo/scripts/test_fsm.py
```

### 4. Remote Controller
Starts the command interface for testing FSM transitions.
```bash
python3 sentry_gazebo/scripts/remote.py
```

Remote Commands:
* 1: Start Match (Navigate to Central Zone)
* 2: Retreat (Navigate to Supply Zone)
* 3: Enemy Detected (Interrupt navigation, stop, and aim)
* 4: Clear Enemy (Resume previous task)

## Dependencies
Install Gazebo Fortress, Navigation2, and SLAM tools
```bash
sudo apt-get install ros-humble-ros-gz \
                     ros-humble-ros-gz-bridge \
                     ros-humble-slam-toolbox \
                     ros-humble-navigation2 \
                     ros-humble-nav2-bringup \
                     ros-humble-teleop-twist-keyboard
```

