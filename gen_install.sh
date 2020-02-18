#!/bin/bash

set -e

## Install ROS core :
echo "BEGINING ROS INSTALL"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 This is now out of date?
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y

if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    ROS_VERSION="kinetic"
    else
    if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    ROS_VERSION="melodic"
fi
fi
echo "installing for ros version: ${ROS_VERSION}"
sudo apt-get install -y ros-${ROS_VERSION}-desktop-full
source /opt/ros/${ROS_VERSION}/setup.bash

[ ! -d "/etc/ros/rosdep/sources.list.d" ] && sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

## Install packages we need:
echo "INSTALLING DEPENDENCIES NOT FOUND IN ROSDEP"
pip install pyqtgraph --user
#I think I have replaced this by adding a symlink:
#python flo_face/teensy/src/serial_coms/computer/python/serial-coms/setup.py install --user
pip install mutagen --user

echo "INSTALLING ROSDEP DEPENDENCIES"
sudo apt-get install python-rosdep -y
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
cd -

echo "INSTALLING REALSENSE"
bash realsense_install.sh

echo "Adding updated webrtcros"
prior=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/RobotWebTools/webrtc_ros.git
cd webrtc_ros/webrtc
touch CATKIN_IGNORE
cd $prior

# build it all
cd ~/catkin_ws && catkin_make
source devel/setup.bash
cd -

## Create a folder for bag files
mkdir ~/flo_data -p
