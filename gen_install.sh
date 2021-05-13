#!/bin/bash

set -e

## Install ROS core :
echo "BEGINING ROS INSTALL"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get -qq update -y
sudo apt-get -qq upgrade -y
sudo apt-get -qq autoremove -y
sudo apt-get -qq clean -y
sudo apt-get -qq update --fix-missing -y
sudo apt-get -qq install -f
sudo apt-get -qq upgrade -y

if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    ROS_VERSION="kinetic"
    else
    if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    ROS_VERSION="melodic"
fi
fi
echo "installing for ros version: ${ROS_VERSION}"
sudo apt-get install -qq -y ros-${ROS_VERSION}-desktop-full
source /opt/ros/${ROS_VERSION}/setup.bash
## install rosmon, it would be weird for this to be in one of the packages:
#sudo apt-get install ros-${ROS_VERSION}-rosmon

sudo apt-get -qq install -y python-rosdep
[ ! -d "/etc/ros/rosdep/sources.list.d" ] && sudo rosdep init -q
rosdep update -q
sudo apt-get -qq install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

## Install packages we need:
echo "INSTALLING DEPENDENCIES NOT FOUND IN ROSDEP"
#pip install pyqtgraph --user
#I think I have replaced this by adding a symlink:
#python flo_face/teensy/src/serial_coms/computer/python/serial-coms/setup.py install --user
# Mutagen has dropped python 2 support. Last supported version was 1.43.0:
sudo apt-get -qq install -y python-pip
pip install 'mutagen==1.43.0' --user -q

echo "INSTALLING ROSDEP DEPENDENCIES"
sudo apt-get -qq install python-rosdep -y
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -q -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv kobuki"
cd -

echo "checking vars"
echo "github_actions: $GITHUB_ACTIONS"
echo "ci: $CI"

if [ "$GITHUB_ACTIONS" = true ]
then
    echo "in github actions, SKIPPING REALSENSE"
else
echo "INSTALLING REALSENSE"
bash realsense_install.sh
fi

echo "Adding updated webrtcros"
prior=$(pwd)
cd ~/catkin_ws/src
git clone --single-branch --branch develop https://github.com/RobotWebTools/webrtc_ros.git
cd webrtc_ros
git checkout a2a19da
cd webrtc
touch CATKIN_IGNORE
cd $prior


echo "Adding rosbridge without unsub"
prior=$(pwd)
cd ~/catkin_ws/src
git clone --single-branch --branch nousub https://github.com/mjsobrep/rosbridge_suite.git
cd $prior

# build it all
cd ~/catkin_ws && catkin_make
source devel/setup.bash
cd -

## Create a folder for bag files
mkdir ~/flo_data -p
