source /opt/ros/kinetic/setup.bash

source /home/odroid/hummingbird_ws/devel/setup.bash
source ~/PX4-FlightX/Tools/setup_gazebo.bash ~/PX4-FlightX ~/PX4-FlightX/build_posix_sitl_tailsitter
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/hummingbird_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX/Tools/sitl_gazebo

# Connect Matlab with ROS
MY_IP=$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI="http://"$ROS_IP":11311"

roslaunch launcher hummingbird_computer.launch
read -p "Press any key to continue... " -n1 -s
