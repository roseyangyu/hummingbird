cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0054 NEW)

project(hummingbird_sitl_gazebo)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_ts_aerodynamic_plugin SHARED src/gazebo_ts_aerodynamic_plugin.cpp)

set(plugins
  gazebo_ts_aerodynamic_plugin
)