#!/bin/bash

set -e
## Node
prior=$(pwd)
cd ~/Downloads
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
cd $prior
