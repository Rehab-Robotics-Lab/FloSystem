#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
function ifip { /sbin/ifconfig $1 | grep "inet addr" | awk -F: '{print $2}' |awk '{print $1}'; }
export ROS_IP=`ifip wlp58s0`
export DISPLAY=:0

tmux new-session -d -s flo
tmux rename-window startup
tmux send-keys 'roscore' Enter

tmux split-window -t flo
tmux send-keys 'mon launch --name=flo_launcher flo_core flo_bringup.launch' Enter

tmux split-window -t flo -h
tmux send-keys 'htop' Enter

tmux split-window -t flo -h

tmux send-keys 'cd ~/catkin_ws/src/LilFloSystem/flo_web/web_app/src/' Enter
#tmux send-keys 'npm run robot' Enter

tmux rotate-window -t flo

tmux new-window -t flo -n audio
tmux send-keys 'alsamixer' Enter
tmux split-window -t flo
tmux send-keys 'pacmd list-sinks|grep index' Enter
tmux send-keys 'pacmd set-default-sink'

tmux select-window -t flo -n
