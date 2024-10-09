cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
  nav_core
  pluginlib
  roscpp
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_msgs tf2_geometry_msgs tf2 nav_core pluginlib roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rrt_planner_ros.cpp
  src/rrt_planner.cpp
  src/collision_detector.cpp
  src/random_double_generator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)