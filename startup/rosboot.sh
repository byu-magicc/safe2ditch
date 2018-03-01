#!/bin/bash
#
# This script gets run via systemctl on boot (if installed)

shopt -s expand_aliases
source /home/nvidia/dev/safe2ditch/dotfiles/tx2rc

# Reset ROS Networking -- NASA doesn't use wifi connection during flight
ROS_IP=127.0.0.1

BAGPATH=$HOME

############################################
# Attempt to get the test name for the bag #
############################################

# Start with an empty bagname
BAGNAME=
if [ -f ".testname" ];
then
	BAGNAME="`cat .testname`.bag"

	# If this test already exists, append the current date
	if [ -f "$BAGPATH/$BAGNAME" ];
	then
		BAGNAME="`cat .testname`-`date '+%Y-%m-%d_%H:%M:%S'`.bag"
	fi
else
	BAGNUM_FILE=.bagnum

	# Read which bag run we are on
	if [ -f "$BAGNUM_FILE" ];
	then
		BAGNUM=`cat ${BAGNUM_FILE}`
	else
		BAGNUM=1
	fi

	# Increment the BAGNUM and save to file
	BAGNUM=$((BAGNUM+1))
	echo ${BAGNUM} > ${BAGNUM_FILE}

fi

# Start the system
rosgo &

# Record a bag
sleep 5
bagrecord -O "$BAGPATH/$BAGNAME"
