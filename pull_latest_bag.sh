#!/bin/sh

set -e
server="nuc-admin@flo-nuc"
file=$(ssh $server 'ls -t ~/flo_data | head -1')
path="flo_data/$file"
echo 'found ' $file
echo 'full path:' $path
scp $server:$path ~/Downloads/
echo 'pulled the latest bag folder into the local downloads folder'
