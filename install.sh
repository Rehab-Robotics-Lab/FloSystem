#!/bin/bash

set -e

## Install ROS core :
echo "BEGINING ROS INSTALL"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update -y
sudo apt install -y ros-kinetic-desktop-full
source /opt/ros/kinetic/setup.bash

[ ! -d "/etc/ros/rosdep/sources.list.d" ] && sudo rosdep init
rosdep update
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
mkdir -p ~/catkin_ws/src
ln -sf ~/Documents/git/LilFloSystem ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make
source devel/setup.bash
cd -

## Install packages we need:
ECHO "INSTALLING DEPENDENCIES NOT FOUND IN ROSDEP"
pip install pyqtgraph --user

echo "INSTALLING ROSDEP DEPENDENCIES"
sudo apt install python-rosdep
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
cd -
