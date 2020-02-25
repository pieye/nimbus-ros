<img src="./doc/images/PIEYE_Logo_RGB_POS.png" align="right"
     title="pieye logo" width="184" height="55">
<img src="./doc/images/IWT.png" align="right"
     title="IWT logo" width="184" height="55">

# Nimbus 3D Camera ROS driver.
Under development

|Branch    | ROS Distro | Status    |
|----------|------------|-----------|
|master    | Melodic    |[![Build Status](https://travis-ci.org/lernfabrik/nimbus-ros.svg?branch=master)](https://travis-ci.org/lernfabrik/nimbus-ros)|

## Installation

1. Raspberry Pi 4 ROS installation
    * Please follow [low-level_implementation](https://github.com/pieye/nimbus-ros/tree/low-level_implementation) branch.
2. Create Catkin Work Place

``` 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/pieye/nimbus-ros.git
cd ..
source /opt/ros/melodic/setup.bash
rosdep update && rosdep install --from-paths --ignore-src src -y
catkin init
catkin build
source devel/setup.bash
```