#!/bin/bash

set -e

pip3 install -U boto3

## Installing AWS CLI
prior=$(pwd)
cd ~/Downloads
curl "https://s3.amazonaws.com/aws-cli/awscli-bundle.zip" -o "awscli-bundle.zip"
unzip awscli-bundle.zip
sudo ./awscli-bundle/install -i /usr/local/aws -b /usr/local/bin/aws
cd ~/Downloads
rm awscli-bundle.zip
rm -r awscli-bundle
cd $prior
aws configure set region us-east-1 --profile flo
aws configure set output json --profile flo
aws configure --profile flo

### Installing boto3 for AWS stuff. Not 100% sure this is needed
# It looks like rosdep now handles this
#sudo apt install -y python3-pip
#pip3 install -U boto3
