#!/bin/bash

source ~/.bashrc

tmux new-session -d -s flo
tmux rename-window startup
tmux send-keys 'roscore' Enter
tmux split-window -t flo -h


tmux split-window -t flo
tmux send-keys 'mkdir -p ~/flo_data && roslaunch --wait flo_core podium_bringup.launch' Enter # it is just more stable..

tmux rotate-window -t flo

tmux split-window -t flo -h
tmux send-keys 'sleep 5 && roslaunch --wait flo_telepresence realsense-sp-1.launch platform:=podium' Enter # it is just more stable..

tmux split-window -t flo
tmux send-keys 'sleep 10 && roslaunch --wait flo_telepresence realsense-sp-2.launch platform:=podium' Enter # it is just more stable..
