#!/bin/bash

set -e
sudo apt-get -qq update && sudo apt-get -qq upgrade && sudo apt-get -qq dist-upgrade
sudo apt-get -qq install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get -qq install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    ROS_VERSION="kinetic"
    OS_VERSION="xenial"
    LIB_REALSENSE_VERSION=2.48.0
    REALSENSE_ROS_VERSION=2.3.1
    else
    if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    ROS_VERSION="melodic"
    OS_VERSION="bionic"
    LIB_REALSENSE_VERSION=2.48.0
    REALSENSE_ROS_VERSION=2.3.1
    else
    if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==20)); then
    ROS_VERSION="noetic"
    OS_VERSION="focal"
    LIB_REALSENSE_VERSION=2.54.1
    REALSENSE_ROS_VERSION=2.3.1
fi
fi
fi

sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "installing for ros version: ${ROS_VERSION} and os version: ${OS_VERSIOIN}"
## Realsense
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo ${OS_VERSION} main" -u
#You need to figure out the versions you want from https://github.com/IntelRealSense/realsense-ros/releases


sudo apt-mark unhold librealsense2-utils
sudo apt-mark unhold librealsense2-dev
sudo apt-mark unhold librealsense2-dbg

sudo apt-get  install -y librealsense2-dkms
sudo apt-get  install -y librealsense2-utils=${LIB_REALSENSE_VERSION}\*
sudo apt-get  install -y librealsense2-dev=${LIB_REALSENSE_VERSION}\*
sudo apt-get  install -y librealsense2-dbg=${LIB_REALSENSE_VERSION}\*
sudo apt-get  install -y ros-${ROS_VERSION}-cv-bridge
sudo apt-get  install -y ros-${ROS_VERSION}-image-transport
sudo apt-get  install -y ros-${ROS_VERSION}-tf
sudo apt-get  install -y ros-${ROS_VERSION}-diagnostic-updater
sudo apt-get  install -y ros-${ROS_VERSION}-ddynamic-reconfigure

sudo apt-mark hold librealsense2-utils
sudo apt-mark hold librealsense2-dev
sudo apt-mark hold librealsense2-dbg

prior=$(pwd)
cd ~/catkin_ws/src
[ ! -d "realsense-ros" ] && git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git fetch
git checkout ${REALSENSE_ROS_VERSION}
cd ~/catkin_ws
catkin_make --only-pkg-with-deps realsense2_camera realsense2_description
catkin_make install
cd $prior
