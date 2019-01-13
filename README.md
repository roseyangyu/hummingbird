# Hummingbird
 [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Copyright &copy;2018 [STARS Lab](http://www.starslab.ca/)

<img src="http://www.starslab.ca/wp-content/themes/stars-lab/images/stars-logo.png" width="500">
<br><br>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Components

#### Mechanical Components
######(hummingbird_design/CAD Files)
An overall mechanical CAD design document, 3D-print-ready model files, and a BOM (Bill of Material) required to assemble Hummingbird is provided under `Hummingbird-MCAD`. For details, refer to the README of the submodule repo.

#### SITL Simulation
######(hummingbird_px4/Tools/sitl_gazebo)
A set of Gazebo Plug-ins required for Hummingbird SITL simulation is provided under `Hummingbird-sitl_gazebo`. This submodule contains a gazebo model description file for the complete Hummingbird model as well as a calibrated aerodynamic model for the vehicle.

#### MATLAB Simulink Model for Tailsitter Dynamics and Motion Control
######(hummingbird_design/Dynamics and Motion Controller)
A high-fidelity MATLAB simulink model is released for easy, quick and accurate simulation of the dynamics and motion control of Hummingbird. The models can be found under `Hummingbird-Simulink`.

#### BLHeli ESC Firmware
###### (hummingbird_px4)
Our modified BLHeli firmware released under `BLHeli` allows DYS-SN20A ESCs to send synchronous pulses at phase commutations to the flight computer which processes the timestamps of these signals to infer motor speeds. Please refer to the README of the repo for compiling and flashing ESC firmware. 

#### PX4 Microcontroller Firmware
###### (hummingbird_firmware/BLHeli)
This repo (`PX4-Hummingbird`) contains all of the flight code (controller, state estimator, state machine, drivers, communication software) running onboard the flight computer. One may use QGroundControl (the default PX4 GCS), or any MAVlink-enabled clients such as MAVROS to communicate with the onboard computer to receive telemtry and send commands.

## Contact
Yilun Wu  <yl.wu@robotics.utias.utoronto.ca>

Jason Wang <jiashen.wang@robotics.utias.utoronto.ca>

Rahman Qureshi <rahman.qureshi@robotics.utias.utoronto.ca>
  
## Citation
If you use any of the resources in academic work, please cite the [relevant publication](TBA): 

```bibtex
@conference{hummingbird,
	Author = {Yilun Wu and Xintong Du and Rikky Duivenvoorden and Jonathan Kelly},
  Title = {Hummingbird: An Open-Source Dual-Rotor Tail-Sitter Platform for Research and Education},
	Booktitle = {2019 International Conference on Robotics and Automation (ICRA)},
	Note = {Submitted, Under review},
	Year = {2019}}
```

## Documentation

#### ROS Software Code Instructions
Welcome to the software repository, to start working on the robot, follow the instructions to install ros

http://wiki.ros.org/ROS/Installation

Debian packages needed for robots (sudo apt-get install)
- git
- git-gui
- python-catkin-tools

IDE recommended
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/)
- CLion Setup https://github.com/ethz-asl/programming_guidelines/wiki/CLion
- Rename jetbrains-clion.desktop to clion.desktop. This way Jetbrains toolbox doesn't override the file when you restart.

First you need to clone the microcontroller firmware and matlab scripts used in the repository

```bash
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/RahmanQureshi/Hummingbird-ROS #  To clone the repository
cd soccer_ws
git checkout initials_branchname
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y # To install all dependencies
sudo ./src/Hummingbird-ROS/hummingbird/scripts/install_geographiclib_datasets.sh # For MAVROS to work
catkin build soccerbot
source devel/setup.bash # Needs to be done everytime you finish building
```

Edit your .bashrc, it should look like this, but you have to run ifconfig to see the correct interface for your Wifi. Replace wlp110s0 with your wifi interface name

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/Hummingbird/PX4-Humminngbird/Tools/setup_gazebo.bash ~/Hummingbird/PX4-Humminngbird ~/Hummingbird/PX4-Humminngbird/build_posix_sitl_tailsitter
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Hummingbird/PX4-Humminngbird/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Hummingbird/PX4-Humminngbird/Tools/sitl_gazebo
MY_IP=$(ifconfig wlp110s0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI=http://$ROS_IP:11311
```

You should be ready to go now. Before running, setup your CLion IDE (above),  To run the robot:

```bash
roslaunch hummingbird hummingbird.launch
```

For simulation you can just run this

```bash
roslaunch hummingbird hummingbird_simulation.launch
```
#### PX4 Firmware Code instructions

```bash
cd ~
git clone --recurse-submodules https://github.com/RahmanQureshi/Hummingbird #  To clone the repository
cd Hummingbird/PX4-Humminngbird
make posix_sitl_tailsitter
```

#### Matlab Code instructions
