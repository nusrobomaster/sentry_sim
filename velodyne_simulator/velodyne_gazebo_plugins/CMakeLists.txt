cmake_minimum_required(VERSION 3.8)
project(velodyne_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz REQUIRED)
# find_package(gazebo_dev REQUIRED)

# Set the C++ standard to C++14 (ROS 2 Humble default)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
# target_include_directories(
#   include
#   ${ros_gz_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )

include_directories(
  include
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create libraries
# add_library(gazebo_ros_velodyne_laser SHARED src/GazeboRosVelodyneLaser.cpp)
# target_link_libraries(gazebo_ros_velodyne_laser
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
#   ${ros_gz_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   RayPlugin
# )

# add_library(gazebo_ros_velodyne_gpu_laser SHARED src/GazeboRosVelodyneLaser.cpp)
# target_link_libraries(gazebo_ros_velodyne_gpu_laser
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
#   ${ros_gz_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   GpuRayPlugin
# )
# target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

add_executable(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)

ament_target_dependencies(gazebo_ros_velodyne_laser ament_cmake rclcpp sensor_msgs tf2_ros ros_gz)

add_executable(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)

ament_target_dependencies(gazebo_ros_velodyne_gpu_laser ament_cmake rclcpp sensor_msgs tf2_ros ros_gz)

# Install the targets
install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser)
ament_export_dependencies(rclcpp sensor_msgs tf2_ros ros_gz)

# Package ament
ament_package()
