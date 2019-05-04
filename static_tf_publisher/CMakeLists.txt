cmake_minimum_required(VERSION 2.8.3)
project(static_tf_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/static_tf_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  static_tf_publisher)

add_library(static_tf_publisher STATIC
  src/static_tf_publisher.cpp)
target_link_libraries(static_tf_publisher
  yaml-cpp
)
