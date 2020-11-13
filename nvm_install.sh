#!/bin/bash

set -e
## Node
prior=$(pwd)
cd ~/Downloads
curl -sL https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh -o install_nvm.sh
bash install_nvm.sh
source ~/.profile
nvm install node
cd $prior

npm install -g pm2
