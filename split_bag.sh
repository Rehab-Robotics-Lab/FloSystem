#!/bin/bash

# provide input bag, output bags prefix, and time fraction
echo $1, $2, $3
t0=`rosbag info -y -k start $1`
t1=`rosbag info -y -k end $1`
tfr=`echo "$t0 + ($t1 - $t0) * $3" | bc -l`
echo $t0, $t1, $tfr
rosbag filter $1 $2_a.bag "t.secs <= $tfr"
rosbag filter $1 $2_b.bag "t.secs > $tfr"
