#!/bin/bash
#
# This script gets run via systemctl on boot (if installed)

shopt -s expand_aliases
source /home/nvidia/dev/safe2ditch/dotfiles/tx2rc

# Reset ROS Networking -- NASA doesn't use wifi connection during flight
ROS_IP=127.0.0.1

BAGNUM_FILE=.bagnum

# Read which bag run we are on
if [ -f "$BAGNUM_FILE" ];
then
	BAGNUM=`cat ${BAGNUM_FILE}`
else
	BAGNUM=1
fi

sleep 15
rosgo &
sleep 15

# Increment the BAGNUM and save to `~/bagnum`
BAGNUM=$((BAGNUM+1))
echo ${BAGNUM} > ${BAGNUM_FILE}

bagrecord -O "run${BAGNUM}.bag"  # this is where you can change the bag path

## Increment the BAGNUM and save to `~/bagnum`
#BAGNUM=$((BAGNUM+1))
#echo ${BAGNUM} > ${BAGNUM_FILE}
