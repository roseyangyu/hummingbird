source /opt/ros/kinetic/setup.bash

source /home/odroid/hummingbird_ws/devel/setup.bash
source ~/PX4-FlightX/Tools/setup_gazebo.bash ~/PX4-FlightX ~/PX4-FlightX/build_posix_sitl_tailsitter
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/hummingbird_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX/Tools/sitl_gazebo

roslaunch launcher hummingbird_gazebo.launch
read -p "Press any key to continue... " -n1 -s
