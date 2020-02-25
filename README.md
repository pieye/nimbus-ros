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
4. Not complete yet....

### .pcd from .stl
Here we require "Meshlab" to convert any given stl model of an object to corresponding point cloud data formate.
1. Open Meshlab and File-> Impor Mesh.. -> `Choose the stl file`.
2. If you press the `Points` tab will show the number of dots on the object.
3. Inoreder to increase the point you need to increase the sampling.
4. Here I have chosen `Poission-disk sampling` (Filters -> Sampling -> Poission-Disk Sampling)
5. Increase the munber of samples (If you increase it too much then it will take some time to create the samples).
6. Now Export Mash as -> `Choose the file formate .ply` -> save
7. Open corresponding file path and run `pcl_ply2pcd <filename>.ply <filename>.pcd` 

### Results
<img src="./doc/images/rviz nimbus point cloud intensity mug.png" align="center"
     title="test 1" width="356" height="286">

### ToDo
1. Create launch file
2. Remove the embedded ip addr. 
