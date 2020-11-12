#!/bin/bash

set -e

sudo apt-get install python3-pip curl git -y

## Adding to the bashrc file
grep -qxF 'source ~/catkin_ws/src/LilFloSystem/bash_includes' ~/.bashrc || echo 'source ~/catkin_ws/src/LilFloSystem/bash_includes' >> ~/.bashrc

bash aws_install.sh

source gen_install.sh

## Kobuki udev rules
rosrun kobuki_ftdi create_udev_rules

bash node_install.sh
