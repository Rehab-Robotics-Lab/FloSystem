#!/bin/bash

set -e

sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install python3-pip curl git -y

## Adding to the bashrc file
grep -qxF 'source ~/catkin_ws/src/FloSystem/bash_includes' ~/.bashrc || echo 'source ~/catkin_ws/src/FloSystem/bash_includes' >> ~/.bashrc

source gen_install.sh

bash aws_install.sh

## Kobuki udev rules
sudo apt-get install -y ros-melodic-kobuki-ftdi
source ~/.bashrc && rosrun kobuki_ftdi create_udev_rules

bash node_install.sh

sudo apt-get install -y tmux

#From: https://github.com/mjsobrep/Turtlebot2-On-Melodic/blob/master/install_basic.sh
echo "Installing Kobuki"
prior=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator.git

git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki.git
mv kobuki/kobuki_description kobuki/kobuki_node \
  kobuki/kobuki_keyop kobuki/kobuki_safety_controller \
  kobuki/kobuki_bumper2pc ./
rm -rf kobuki

git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki_desktop.git
mv kobuki_desktop/kobuki_gazebo_plugins ./
rm -rf kobuki_desktop

git clone https://github.com/yujinrobot/yujin_ocs.git
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers .
rm -rf yujin_ocs

sudo apt-get install ros-melodic-kobuki-* -y
sudo apt-get install ros-melodic-ecl-streams -y

# necessary for build and gazebo
sudo apt-get install ros-melodic-depthimage-to-laserscan -y
sudo apt-get install ros-melodic-joy -y
sudo apt-get install ros-melodic-yocs-velocity-smoother -y

echo "Installing Vid Stream Opencv"
git clone https://github.com/ros-drivers/video_stream_opencv.git

cd ~/catkin_ws
rm -rf build devel install
catkin_make
catkin_make install
cd $prior
