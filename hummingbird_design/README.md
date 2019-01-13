# hummingbird_matlab

There are 3 things you need to setup. 

* PX4-FlightX repository (Microcontroller and simulator Code)
* hummingbird_ws repository (Main Computer Code (Computer Vision)
* hummingbird_matlab repository (Remote Computer Navigation Testing)

### Set Up

1. Clone the hummingbird_ws in the Odroid computer or the main computer code you are using
2. Copy paste this into your .bashrc. This will enable your ros to be able to connect the simulator
```sh
# Connect ROS with simulator
source /home/vuwij/hummingbird_ws/devel/setup.bash
source ~/PX4-FlightX/Tools/setup_gazebo.bash ~/PX4-FlightX ~/PX4-FlightX/build_posix_sitl_tailsitter
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/hummingbird_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-FlightX/Tools/sitl_gazebo

# Connect Matlab with ROS
MY_IP=$(ifconfig wlp4s0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI="http://"$ROS_IP":11311"
```
3. Run by typing
```sh
roslaunch px4 mavros_posix_sitl.launch
```
4. On the Matlab computer clone the hummingbird_matlab folder. In the loadRobot.m file change the following information
```matlab
gazeboIp = '192.168.2.21'; # The IP address of the Odroid
localIp = '192.168.2.14'; # Your local computer IP address (matlab)
```
5. Run the auto_takeoff.m file to take off directly from matlab

6. Open hummingbird.slx and click run to start tracking the apriltag
