#!/bin/bash

set -e

## Adding to the bashrc file
grep -qxF 'source ~/catkin_ws/src/LilFloSystem/bash_includes' ~/.bashrc || echo 'source ~/catkin_ws/src/LilFloSystem/bash_includes' >> ~/.bashrc

## Installing AWS CLI
prior=$(pwd)
cd ~/Downloads
curl "https://s3.amazonaws.com/aws-cli/awscli-bundle.zip" -o "awscli-bundle.zip"
unzip awscli-bundle.zip
sudo ./awscli-bundle/install -i /usr/local/aws -b /usr/local/bin/aws
rm aswcli-bundle.zip
rm -rf awscli-bundle
cd $prior

### Installing boto3 for AWS stuff. Not 100% sure this is needed
# It looks like rosdep now handles this
#sudo apt install -y python3-pip
#pip3 install -U boto3

source gen_install.sh

## Kobuki udev rules
rosrun kobuki_ftdi create_udev_rules

## Realsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install -y librealsense2-dkms
sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev
sudo apt-get install -y librealsense2-dbg

prior=$(pwd)
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd $prior
