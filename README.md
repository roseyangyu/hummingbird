# Hummingbird
 [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Travis build](https://travis-ci.org/utra-robosoccer/soccer_ws.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer_ws)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer_ws.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer_ws/alerts/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws/badge.svg)](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws)

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
Jason Wang <jiashen.wang@robotics.utias.utoronto.ca>

Yilun Wu  <yl.wu@robotics.utias.utoronto.ca>

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


Welcome to the software repository, to start working on the robot, follow the instructions to install ros

http://wiki.ros.org/ROS/Installation

#### Prerequisites

Debian packages needed for robots (sudo apt-get install)
- git
- git-lfs
- python-catkin-tools
- net-tools
- indicator-ip

#### Setting up your IDE
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/)
- Follow the CLion Setup here, use method 2 to add bash to the launch file https://github.com/ethz-asl/programming_guidelines/wiki/CLion
- In CLion, once you finish following the instructions, you should be able to reload CMake to have code hinting enabled
- Install the *.launch file plugins if you want to. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Add the python2.7 intepretor to CLion to get Clion code hinting. In Settings/Build,Execution,Deployment/Python Intepretor, add the system intepretor /usr/bin/python 2.7
- For debugging processes follow the steps here https://www.jetbrains.com/help/clion/attaching-to-local-process.html

#### Initialization of the code
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
sudo apt-get install python-catkin-tools # If you don't have the package installed yet.
catkin_init_workspace
git clone --recurse-submodules https://github.com/utiasSTARS/hummingbird #  To clone the repository
git lfs init
git lfs pull
cd hummingbird
git checkout initials_branchname  # TO create a new branch, use git checkout -b initials_branchname
cd ~/catkin_ws
```
#### Installing submodules and dependencies
```
cd ~/catkin_ws/src/hummingbird
git submodule update --recursive --init
sudo rosdep init # If first time using ROS in your environment.
rosdep update
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic # To install all dependencies (use correct ROS distro version), add --os ubuntu:xenial if your linux is based on it but has different distro name and version. Ubuntu 16.04 uses kinetic instead of melodic. For Jetson TX2 use kinetic.
```

#### Building the code
```
catkin build hummingbird # Use catkin clean to start with a clean build
source devel/setup.bash # Needs to be done everytime you finish building
```

Build and run tests
```
catkin build <pkg name> --verbose --catkin-make-args run_tests
```

#### Launching the robot
You should be ready to go now. Before running, setup your CLion IDE (above),  To run the robot:

```bash
roslaunch hummingbird hummingbird.launch
```

For simulation you can just run this

```bash
roslaunch hummingbird hummingbird.launch simulation:=true
```
