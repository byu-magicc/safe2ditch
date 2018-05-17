#!/bin/bash
#
# This script gets run via systemctl on boot (if installed)

# Get path to the directory of this file, no matter where it is sourced from
MYDIR=$(dirname ${BASH_SOURCE[0]})

# Setup the Safe2Ditch environment
shopt -s expand_aliases
source "$MYDIR/s2denv"

# Wait a bit for networking
sleep 5

# If network still hasn't connected, then just force local IP
# (i.e., if ROS_IP is not 127.0.0.1 and there is no network,
#  then ROS will never actually start up)
ping -c 1 $ROS_IP > /dev/null
if [ $? -ne 0 ]; then
	# Could not find $ROS_IP, force local
	export ROS_IP=127.0.0.1
fi

# Start the system
roslaunch nasa_s2d hardware.launch &

# Increment the TESTNUM and save to file
TESTNUM=$(cat $S2DENV/testnum)
TESTNUM=$((TESTNUM+1))
echo $TESTNUM > "$S2DENV/testnum"

# Record a bag
sleep 5
bagrecord -O "$S2DBAGS/$S2DTESTNAME.bag"
