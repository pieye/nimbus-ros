cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(nimbus_3d_driver VERSION 0.10)

## Use C++11
add_definitions(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-Ofast -march=native -ftree-parallelize-loops=4 -DNDEBUG)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   #${PROJECT_NAME}_node
   CATKIN_DEPENDS geometry_msgs pcl_ros roscpp sensor_msgs std_msgs tf2
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

## Declare a cpp library
add_library(${PROJECT_NAME}_node
   src/Algorithm.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/nimbus_3d_driver.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} /usr/local/lib/libnimbus-preprocessing.so)
