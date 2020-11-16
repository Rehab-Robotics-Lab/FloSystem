#!/bin/bash

set -e
## Node
prior=$(pwd)
cd ~/Downloads
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
cd $prior

## install some dependencies
NPM_CONFIG_PREFIX=~/.npm-global

npm install -g pm2

prior=$(pwd)
cd ~/catkin_ws/src/LilFloSystem/flo_web/webrtc_robot_pinger
npm install
cd $prior
