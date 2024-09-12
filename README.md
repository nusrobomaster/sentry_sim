# sentry_sim

Sentry simulation on gazebo. Modified form the original forked library to make it compatiable for ROS 2 Humble with ignition gazebo fortress.

## Progress
Currently able to launch RMUC arena in gazebo and spawn the robot from xacro file (arbitrary model) with correct physics. **Next goal is to debug code for controlling the sentry in the simulation with key controls.**

Latest test launching from `gazebo_rmuc_test_launch.py` with urdf from `test_robot.xacro`.

## Launch files
Simulation launch files are located in `sentry_gazebo`. 

## Launch Simulation
```SHELL
./build_packages.sh
source install/setup.bash
ros2 launch sentry_gazebo gazebo_rmuc_test_launch.py
```

## Packages
Work in progress...

## Dependencies
Set up gazebo fortress (might need to install binaries first)
```SHELL
sudo apt-get install ros-humble-ros-gz
sudo apt-get install ros-humble-ros-ign-bridge
```

[autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)

https://github.com/66Lau/sentry_sim/assets/95697190/59206443-fcca-4397-8dfb-3bf6a5fa4ec9

Some files in `autonomous_exploration_development_environment`, `far_planner` package are modified.

<!-- https://github.com/66Lau/sentry_sim/assets/95697190/a8513286-9576-4109-98dd-e6898c791bb9

https://github.com/66Lau/sentry_sim/assets/95697190/b711e6e7-a677-423d-bf40-183172f097cb -->

<div align="center"><img src="img/slope_img.png" width=90% /></div>
<div align="center">Uphill</div>
<br>

<div align="center"><img src="img/sim_img.png" width=90% /></div>
<div align="center">Navigation in bumopy road</div>
<br>

<div align="center"><img src="img/far_planner.png" width=90% /></div>
<div align="center">Global navigation(far_planner)</div>
<br>


## Reference
Appreciate!
- [CMU-autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)
- [CMU-FAR-planner](https://github.com/MichaelFYang/far_planner)
- [KDRobot_RM2023Sentry_Navigation](https://github.com/Wangben1019/KDRobot_RM2023Sentry_Navigation)
- [Hbut_LC_sentry](https://github.com/HBUTHUANGPX/Hbut_LC_sentry)

## Q&A


<div align="center"><img src="img/problem1.png" width=90% /></div>
<div align="center">Drop</div>
<br>

When lidar are not able to scan the ground below the cliff, the system would grant cliff passable. It could be solved by set "noDataObstacle" to true.

<!-- 
调参理解

local_planner.launch 中的 twoWayDrive 在rm场景下没有必要允许双头运行，有时候会导致摇摆
terrain_analysis中的 useSorting 打开比较好，因为那样子对于地面的划分就不会过于依靠绝对高度，而是会在已知点云中按照高度排序，划分地面层
terrain_analysis中的 maxGroundLift 在对点云分类到障碍物或者地面起作用，这个值越大，能上越陡的坡

第一套：原始参数
第二套：twoWayDrive->false
        minRelZ->-2.5
        maxGroundLift->2.0
        useSorting->true
TODO:
使用普通的A*的全局路径规划，然后base_planner使用cmu的框架，目前还差从path中提取waypoint


roslaunch sentry_global_planner sentry_global_planner.launch 



 -->