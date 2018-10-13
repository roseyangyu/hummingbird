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
  geometry_msgs
  apriltags2_msgs
  apriltags2_ros
  tf2
  tf2_ros
)

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
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/odometry_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(ts_location_estimator src/ts_location_estimator.cpp)
add_dependencies(ts_location_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ts_location_estimator
  ${catkin_LIBRARIES}
)

add_executable(ts_location_publisher src/ts_location_publisher.cpp)
add_dependencies(ts_location_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ts_location_publisher
  ${catkin_LIBRARIES}
)
