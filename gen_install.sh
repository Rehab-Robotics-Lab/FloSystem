#!/bin/bash

set -e

## Install ROS core :
echo "BEGINING ROS INSTALL"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get -qq update -y

if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==16));then
    readonly ROS_VERSION_TARGET="kinetic"
fi    
if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==18)); then
    readonly ROS_VERSION_TARGET="melodic"
fi
if(($(cat /etc/os-release | grep VERSION_ID|grep -o '".*"' | sed 's/"//g' | cut -c1-2 )==20)); then
    readonly ROS_VERSION_TARGET="noetic"
fi
echo "installing for ros version1: ${ROS_VERSION_TARGET}"
#to get the variable into global scope
export $ROS_VERSION_TARGET
echo "installing for ros version2: ${ROS_VERSION_TARGET}"
sudo apt-get install -qq -y ros-${ROS_VERSION_TARGET}-desktop-full
echo "installing for ros version3: ${ROS_VERSION_TARGET}"
source /opt/ros/${ROS_VERSION_TARGET}/setup.bash
echo "installing for ros version4: ${ROS_VERSION_TARGET}"
## install rosmon, it would be weird for this to be in one of the packages:
#sudo apt-get install ros-${ROS_VERSION_TARGET}-rosmon
echo "ros version:${ROS_VERSION_TARGET}"
if [[ "$ROS_VERSION_TARGET" == "kinetic" || "$ROS_VERSION_TARGET" == "melodic" ]] 
then
    echo "In ROS Version kinetic or melodic"
    sudo apt-get -qq install -y python-rosdep 
    sudo apt-get -qq install -y python-pip
    #Mutagen has dropped python 2 support. Last supported version was 1.43.0: mutagen moved into if statement to account for ubuntu 20, also python3 supports latest version of mutagen.
    pip install 'mutagen==1.43.0' --user -q
    echo "INSTALLING ROSDEP DEPENDENCIES"
    sudo apt-get -qq install python-rosdep -y
    [ ! -d "/etc/ros/rosdep/sources.list.d" ] && sudo rosdep init -q
    rosdep update -q
    sudo apt-get -qq install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -q -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv kobuki webrtc_ros"
cd -
fi
if [[ "$ROS_VERSION_TARGET" == "noetic" ]] 
then
    echo "In ROS Version noetic"	
    sudo apt-get -qq install -y python3-rosdep
    sudo apt-get -qq install -y python3-pip
    pip install 'mutagen' --user -q
    echo "INSTALLING ROSDEP DEPENDENCIES"
    sudo apt-get -qq install python3-rosdep -y
    [ ! -d "/etc/ros/rosdep/sources.list.d" ] && sudo rosdep init -q
    rosdep update -q
    sudo apt-get -qq install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -q -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv kobuki webrtc_ros"
cd -
fi

## Install packages we need:
echo "INSTALLING DEPENDENCIES NOT FOUND IN ROSDEP"
#pip install pyqtgraph --useri
#I think I have replaced this by adding a symlink:
#python flo_face/teensy/src/serial_coms/computer/python/serial-coms/setup.py install --user

#cd ~/catkin_ws
#rosdep install --from-paths src --ignore-src -q -r -y --skip-keys "realsense2_camera realsense2_description rosbridge_suite rosbridge_server rosbridge_library rosbridge_msgs video_stream_opencv kobuki webrtc_ros"
#cd -

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
