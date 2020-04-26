#!/bin/bash

source ~/.bashrc

tmux new-session -d -s flo
tmux rename-window startup
tmux send-keys 'roscore' Enter

tmux split-window -t flo
#tmux send-keys 'mon launch --name=flo_launcher flo_core flo_bringup.launch' Enter
tmux send-keys 'roslaunch --wait flo_core flo_bringup.launch' Enter # until rosmon gets fixed

tmux split-window -t flo -h
tmux send-keys 'htop' Enter

tmux split-window -t flo -h

tmux send-keys 'cd ~/catkin_ws/src/LilFloSystem/flo_web/webrtc_robot_router/' Enter
tmux send-keys 'npm run tsc && node ./build/app.js' Enter
# need to make this use a build version

tmux rotate-window -t flo

tmux new-window -t flo -n audio
tmux send-keys 'alsamixer' Enter
tmux split-window -t flo
tmux send-keys 'pacmd list-sinks|grep index' Enter
tmux send-keys 'pacmd set-default-sink'

tmux select-window -t flo -n
