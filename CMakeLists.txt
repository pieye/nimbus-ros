cmake_minimum_required(VERSION 2.8.3)
project(nimbus_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES nimbus_ros_driver
   CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} /usr/local/lib/libnimbus-preprocessing.so)