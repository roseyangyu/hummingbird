# Install ROS & catkin_tools
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-catkin-tools

# Install intel realsense drivers and ROS node
wget -O enable_kernel_sources.sh https://raw.githubusercontent.com/IntelRealSense/librealsense/rosdebian/scripts/enable_kernel_sources.sh
bash ./enable_kernel_sources.sh
sudo apt-get install 'ros-*-realsense-camera'

# Download apriltags ROS packages
sudo apt-get install git

# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/utiasSTARS/hummingbird_ws
catkin build apriltags2_ros

echo "export ROS_IP=`hostname -I`" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://roscore_computer_ip_here:11311" >> ~/.bashrc
