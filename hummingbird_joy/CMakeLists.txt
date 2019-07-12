cmake_minimum_required(VERSION 2.8.3)
project(hummingbird_joy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  joy
  mavros_msgs
  sensor_msgs
)

# catkin_python_setup()

catkin_package()
