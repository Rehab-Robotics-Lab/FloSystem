#!/bin/bash

set -e
## Realsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install -y librealsense2-dkms
sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev
sudo apt-get install -y librealsense2-dbg
sudo apt install -y ros-kinetic-cv-bridge
sudo apt install -y ros-kinetic-image-transport
sudo apt install -y ros-kinetic-tf
sudo apt install -y ros-kinetic-diagnostic-updater
sudo apt install -y ros-kinetic-ddynamic-reconfigure

prior=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ~/catkin_ws
catkin_make
catkin_make install
cd $prior
