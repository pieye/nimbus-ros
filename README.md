<img src="./assets/PIEYE_Logo_RGB_POS.png" align="right" title="pieye logo" width="130" >

# Nimbus 3D - ROS driver.

# Nimbus 3D - ROS driver.

## 1. Installation

* [Install](https://github.com/pieye/nimbus-userland) the required software packages for nimbus
* Clone this Repository in the src folder of your ROS workspace 
``` 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch=low-level_implementation https://github.com/pieye/nimbus-ros.git
``` 
* [Install ROS Melodic from Source](http://wiki.ros.org/melodic/Installation/Source)  on manually OR run the following install script: 
``` 
cd nimbus-ros/scripts
sudo ./install.sh
``` 
* Build `nimbus-ros`
``` 
cd ~/catkin_ws
catkin_make
```
    
## 2. Configure [ROS to run accros multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

The following diagram shows the possible architectures for using your Nimbus 3D. The ROS driver "nimbus_ros" is running on the Raspberry Pi and publishes the pointcloud. In this guide the ROS master is also running  on the Pi, but it could run on any other machine in your local network. The Pointcloud is afterwards visualized on another Computer with a Display connected e.g. Laptop. Your algorithms to process the captured data can run locally on your Raspberry or any other device in the local network. 


<img src="./assets/nimbus_ros.png" align="center">


* We now configure ROS to run the master on the Raspberry and access the data via another machine running ROS Melodic with RVIZ installed
* Add this line to the .bashrc of your other machine (laptop), after adapting the IP to your Raspberry Pi if you are using Linux:
```
nano ~/.bashrc
export ROS_MASTER_URI=http://192.168.1.1:11311
```
If you are using Windows you need to set it up as an enviroment variable:
```
Name: ROS_MASTER_URI      Value: http://192.168.1.1:11311
```
* SSH into your Raspberry and run:
```
roscore
```
* Start RVIZ on your machine:
```
rviz
```
It should start if everything works as expected.
    


## 3. Start the Nimbus ROS Driver
* The given launch file starts the nimbus node and a static coordinate transform after executing it on the Raspberry.
```
    source devel/setup.bash 
    roslaunch nimbus_ros nimbus.launch
  ```  
* It is possible to adjust the topics where the Pointcloud, Intensity Image, and Range Image are published. Simply set a new topic name in the launch file. This is necessary when using multiple Nimbus cameras in your local network at the same time.

* Starting RVIZ on any correcly setup device with a monitor, a poincloud and two images should be visible as shwon here:
<img src="./assets/nimbus_ros.gif" align="center">


## 4. Configure the driver to your needs
it is possible to adjust the parameters that have an impact on the amount of transmitted data.
* A 1GBit/s ethernet connection to the Raspberry Pi is highly recommended. If this is given you can launch the default configuration without making any changes.
* If you only have a 100MBit/s Interface you can load the given preset by changing the default.yaml to fast_ethernet.yaml in the launch file (launch/nimbus.launch). This will reduce the resolution!
* If you need to reduce the bandwitdh even further (e.g. wifi) but still need a reliable point cloud, you can replace the config against the low_bandwitdh.yaml This will heavily reduce the resolution!
* Furthermore it is possible to adjust the parameters to your own needs.

## 5. Start developing your own Software for the Nimbus 3D
In addition to the this ROS driver template packages for your future C++ or Python software is included in the package "nimbus_example_c". You can run it by executing:
```
rosrun nimbus_example_c example
```
It includes basic ROS functionallity to start your development.