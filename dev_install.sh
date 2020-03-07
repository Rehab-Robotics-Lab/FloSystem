#!/bin/bash

set -e

sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install lsyncd

mkdir -p ~/catkin_ws/src
ln -sf ~/Documents/git/LilFloSystem ~/catkin_ws/src/
ln -sf ~/Documents/git/tts-ros1 ~/catkin_ws/src/

source gen_install.sh

echo "** Did not install AWS CLI, you will need that for TTS **"
echo "** Did not install Kobuki UDEV, you will need that to use Kobuki **"
