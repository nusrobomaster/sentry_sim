cmake_minimum_required(VERSION 3.8)
project(sentry_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

# add_library(sentry_controller SHARED
#   src/sentry_control_key.cpp)

# target_include_directories(include/sentry_controller PUBLIC)
# target_include_directories(include/sentry_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sentry_controller>
#   $<INSTALL_INTERFACE:include/sentry_controller>  # <prefix>/include/mylib
# )
# add_library(sentry_control_key SHARED src/sentry_control_key.cpp)

add_executable(sentry_control_key src/sentry_control_key.cpp)

ament_target_dependencies(sentry_control_key rclcpp std_msgs geometry_msgs)


#############
## Install ##
#############

install(TARGETS
  sentry_control_key
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )


#############
## Testing ##
#############

ament_export_dependencies(
  rclcpp
)

ament_export_include_directories(include)

ament_package()
