#!/bin/bash
set -o errexit
set -o pipefail

RQT_USER=$(id -u "$USER"):$(id -g "$USER")
export RQT_USER
docker-compose --build -f docker-compose-robot-sim.yml up
