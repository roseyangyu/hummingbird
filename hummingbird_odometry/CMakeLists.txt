cmake_minimum_required(VERSION 2.8.3)
project(hummingbird_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  apriltag_ros
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node
  ${catkin_LIBRARIES}
)

add_executable(ts_location_estimator src/ts_location_estimator.cpp)
target_link_libraries(ts_location_estimator
  ${catkin_LIBRARIES}
)
