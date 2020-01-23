<img src="./assets/PIEYE_Logo_RGB_POS.png" align="right" title="pieye logo" width="130" >

# Nimbus 3D - ROS driver.

## 1. Installation

* Clone this Repository in the src folder of your ROS workspace 
``` 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/pieye/nimbus-userland.git
``` 

* [Install](https://github.com/pieye/nimbus-userland/blob/master/ME.md) the required software packages for nimbus
* [Install ROS Melodic from Source](https://github.com/curl/curl)  on ian OR run the following install script: 
``` 
sudo ./install.sh
``` 
    
## 2. Configure [ROS to run accros multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

The following diagram shows the possible architectures for using your Nimbus 3D. The ROS driver "nimbus_ros" is running on the Raspberry Pi and publishes the pointcloud. In this guide the ROS master is also running  on the Pi, but it could run on any other machine in your local network. The Pointcloud is afterwards visualized on another Computer with a Display connected e.g. Laptop. Your algorithms to process the captured data can run locally on your Raspberry or any other device in the local network. 


<img src="./assets/nimbus_ros.png" align="center">


* We now configure ROS to run the master on the Raspberry and access the data via another machine running ROS Melodic with RVIZ installed
* Add this line to the .bashrc of your other machine (laptop), after adapting the IP to your Raspberry Pi
```
nano ~/.bashrc
export ROS_MASTER_URI=http://192.168.1.1:11311
```
* SSH into your Raspberry and run:
```
roscore
```
* Start RVIZ on your laptop:
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