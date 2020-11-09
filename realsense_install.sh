#!/bin/bash

set -e

if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    ROS_VERSION="kinetic"
    OS_VERSION="xenial"
    else
    if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    ROS_VERSION="melodic"
    OS_VERSION="bionic"
fi
fi

sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
echo "installing for ros version: ${ROS_VERSION}"
## Realsense
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo ${OS_VERSION} main" -u
#You need to figure out the versions you want from https://github.com/IntelRealSense/realsense-ros/releases
LIB_REALSENSE_VERSION=2.39.0
REALSENSE_ROS_VERSION=2.2.18
sudo apt-get install -y librealsense2-dkms
sudo apt-get install -y librealsense2-utils=${LIB_REALSENSE_VERSION}\*
sudo apt-get install -y librealsense2-dev=${LIB_REALSENSE_VERSION}\*
sudo apt-get install -y librealsense2-dbg=${LIB_REALSENSE_VERSION}\*
sudo apt-get install -y ros-${ROS_VERSION}-cv-bridge
sudo apt-get install -y ros-${ROS_VERSION}-image-transport
sudo apt-get install -y ros-${ROS_VERSION}-tf
sudo apt-get install -y ros-${ROS_VERSION}-diagnostic-updater
sudo apt-get install -y ros-${ROS_VERSION}-ddynamic-reconfigure

sudo apt-mark hold librealsense2-utils
sudo apt-mark hold librealsense2-dev
sudo apt-mark hold librealsense2-dbg

prior=$(pwd)
cd ~/catkin_ws/src
[ ! -d "realsense-ros" ] && git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
#git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
git checkout ${REALSENSE_ROS_VERSION}
cd ~/catkin_ws
catkin_make
catkin_make install
cd $prior
