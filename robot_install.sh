#!/bin/bash

set -e

## Adding to the bashrc file
grep -qxF 'source ~/catkin_ws/src/LilFloSystem/bash_includes' ~/.bashrc || echo 'source ~/catkin_ws/src/LilFloSystem/bash_includes' >> ~/.bashrc

source gen_install.sh
