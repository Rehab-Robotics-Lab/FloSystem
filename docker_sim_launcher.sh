#!/bin/bash
set -o errexit
set -o pipefail

rebuild=false

while getopts :r flag
do
    case "${flag}" in
        r) rebuild=true;;
        :) echo 'missing argument' >&2; exit 1;;
        \?) echo 'invalid option' >&2; exit 1
    esac
done

XSOCK=/tmp/.X11-unix
export XSOCK
XAUTH=/tmp/.docker.xauth
export XAUTH
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

ROS_USER=$(id -u "$USER"):$(id -g "$USER")
export ROS_USER
R_GID=$(id -g "$USER")
R_UID=$(id -u "$USER")
export R_GID
export R_UID

pactlmod=$(pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket)

trap 'pactl unload-module "$pactlmod"' EXIT

if [ "$rebuild" = true ] ; then
    docker-compose  -f docker-compose-robot-sim.yml build
fi
docker-compose -f docker-compose-robot-sim.yml up
