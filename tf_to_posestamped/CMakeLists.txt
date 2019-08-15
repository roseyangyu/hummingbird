cmake_minimum_required(VERSION 2.8.3)
project(tf_to_posestamped)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
##########

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/tf_to_posestamped_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)