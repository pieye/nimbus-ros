<img src="./assets/PIEYE_Logo_RGB_POS.png" align="right" title="pieye logo" width="220" >

# Nimbus 3D - ROS driver.

## 0. Preparation
The nimbus-server-ros package is ment to be used as a simple and easy to use remote ROS driver. It doesn't require ROS on the Raspberry Pi and needs no extensive setup. Just a ROS noetic installation on your local machine. The image data is being received via the nimbus-server (websockets + JSON RPC). 

For advanced and more efficient usage of ROS, the [embedded ROS driver](https://github.com/pieye/nimbus-ros.git) is reccomended to be used. 

A raspberry Pi4 is highly recommended, because only it has a real 1GBit/s ethernet interface, which is needed for high frame rates. In addition, the Pi4 is recommended for local image processing, since only the Pi4 has sufficient CPU resources left.

The following step (1. Installation) is only required if you want to set it up yourself.
Otherwise use our prepared [Raspberry Pi OS (buster)](https://cloud.pieye.org/index.php/s/nimbus3D) images.

Fruthermore here is the official [documentation](https://nimbus-docs.readthedocs.io/en/latest/index.html).


## 1. Installation

* [Install](https://github.com/pieye/nimbus-userland) the required software packages for nimbus
* Clone this Repository in the src folder of your ROS workspace 
* You need ROS noetic and [Nimbus-Python](https://github.com/pieye/nimbus-python) on your local machine
``` 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/pieye/nimbus-ros.git
``` 

    
## 2. Start the Nimbus ROS Driver
* The given launch file starts the nimbus node and a static coordinate transform. The IP needs to be changed to your setup.
```
    source devel/setup.bash 
    roslaunch nimbus-ros nimbus_ros.launch ip:=192.168.0.69
```  
* It is possible to adjust the topics where the Pointcloud, Intensity Image are published. Simply set a new topic name in the launch file. This is necessary when using multiple Nimbus cameras in your local network at the same time.

* Starting RVIZ on any correcly setup device with a monitor, a poincloud and an intensity image should be visible.


## 3. Configure the driver to your needs
Depending on the given setup it might be useful to adjust the auto exposure.
If objects are moving fast or a minimum framerate should be achieved it can help do disable hdr and set a max value for the exposure time. The desired exposure can also be adjusted in the config file.