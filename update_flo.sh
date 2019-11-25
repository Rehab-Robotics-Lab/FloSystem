#!/bin/sh

set -e

ssh -t nuc-admin@flo-nuc "sudo apt update -y && sudo apt upgrade -y
"
ssh -T nuc-admin@flo-nuc<< EOSSH
source catkin_ws/devel/setup.sh
cd catkin_ws/src/realsense-ros
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
echo 'pulled latest version of realsense'
cd ../tts-ros1
git pull
echo 'pulled latest version of tts-ros'
cd ../..
rosdep update
rosdep install --from-paths ~/catkin_ws/src/LilFloSystem/ --ignore-src -r -y
catkin_make clean
catkin_make
catkin_make install
echo 'remade the catkin ws'
EOSSH
