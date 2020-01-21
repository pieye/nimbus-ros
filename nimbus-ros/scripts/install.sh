#!/usr/bin/env sh
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
sudo rosdep init
rosdep update
cd ~/catkin_ws/
rosinstall_generator ros_comm --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall
wstool init -j8 src melodic-ros_comm.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/install_isolated/setup.bash
echo 'source ~/catkin_ws/install_isolated/setup.bash' >> ~/.bashrc

