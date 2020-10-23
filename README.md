<img src="./doc/images/PIEYE_Logo_RGB_POS.png" align="right"
     title="pieye logo" width="184" height="55">
<img src="./doc/images/IWT.png" align="right"
     title="IWT logo" width="184" height="55">

# Nimbus 3D Camera ROS driver.
Under development

## Installation

1. Required Packages
    * [Curl](https://github.com/curl/curl)
    * jsoncpp
    * [Web Socket](https://github.com/zaphoyd/websocketpp.git)
    * [Install ROS Melodic for Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)
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
3. Run the Node
    * Change the ip addr in `main.cpp`(This will be change in future).
    * `rosrun nimbus_ros nimbus_ros_node`

### ToDo
1. Create launch file
2. Remove the embedded ip addr. 
