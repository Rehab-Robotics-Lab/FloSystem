#!/bin/bash

set -e

mkdir -p ~/catkin_ws/src
ln -sf ~/Documents/git/LilFloSystem ~/catkin_ws/src/
ln -sf ~/Documents/git/tts-ros1 ~/catkin_ws/src/

source gen_install.sh
