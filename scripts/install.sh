#!/usr/bin/env sh
sudo apt-get install python-pip python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
sudo apt-get install python-setuptools python-empy python-defusedxml python-netifaces
sudo rosdep init
rosdep update
mkdir ~/ros_install_ws/
cd ~/ros_install_ws/
rosinstall_generator perception --rosdistro melodic --deps --wet-only --tar > melodic-perception-wet.rosinstall
wstool init src melodic-perception-wet.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
source ~/ros_install_ws/install_isolated/setup.bash
echo 'source ~/ros_install_ws/install_isolated/setup.bash' >> ~/.bashrc
