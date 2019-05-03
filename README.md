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
######hummingbird_design/CAD Files
An overall mechanical CAD design document, 3D-print-ready model files, and a BOM (Bill of Material) required to assemble Hummingbird is provided under `Hummingbird-MCAD`. For details, refer to the README of the submodule repo.

#### SITL Simulation
######hummingbird_px4/Tools/sitl_gazebo
A set of Gazebo Plug-ins required for Hummingbird SITL simulation is provided under `Hummingbird-sitl_gazebo`. This submodule contains a gazebo model description file for the complete Hummingbird model as well as a calibrated aerodynamic model for the vehicle.

#### MATLAB Simulink Model for Tailsitter Dynamics and Motion Control
######hummingbird_design/Dynamics and Motion Controller
A high-fidelity MATLAB simulink model is released for easy, quick and accurate simulation of the dynamics and motion control of Hummingbird. The models can be found under `Hummingbird-Simulink`.

#### BLHeli ESC Firmware
######hummingbird_px4
Our modified BLHeli firmware released under `BLHeli` allows DYS-SN20A ESCs to send synchronous pulses at phase commutations to the flight computer which processes the timestamps of these signals to infer motor speeds. Please refer to the README of the repo for compiling and flashing ESC firmware. 

#### PX4 Microcontroller Firmware
######hummingbird_firmware/BLHeli
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

#### Getting Started
Welcome to the software repository, to start working on the robot, use the [PX4 Setup Script](https://github.com/PX4/Devguide/blob/master/build_scripts/ubuntu_sim_ros_gazebo.sh) to install
ROS Kinetic and gazebo, as well as setup a catkin workspace with mavros and mavlink inside it. 

Next, clone this repository into catkin_ws/src and build the code using the following instructions.

```bash
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/RahmanQureshi/hummingbird_ws #  To clone the repository
cd ~/catkin_ws
catkin build
source devel/setup.bash # Needs to be done everytime you finish building
```

Append the following to your .basrc, but you have to run ifconfig to see the correct interface for your Wifi. Replace wlp110s0 with your wifi interface name

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/src/hummingbird_ws/hummingbird_px4/Tools/setup_gazebo.bash ~/catkin_ws/src/hummingbird_ws/hummingbird_px4 ~/catkin_ws/build/hummingbird_px4 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src
MY_IP=$(ifconfig wlp59s0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

```

Source your bashrc. You should be ready to go now. To run the robot:

```bash
roslaunch hummingbird hummingbird.launch
```

For simulation you can just run this

```bash
roslaunch hummingbird hummingbird_simulation.launch
```

#### Matlab Code instructions

See `hummingbird_ws/hummingbird_design/Docking Simulink` for useful scripts.

Install the custom ROS messages for MATLAB in `hummingbird_design/Docking Simulink/msgs`