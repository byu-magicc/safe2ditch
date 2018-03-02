#!/bin/bash
#
# This script gets run via systemctl on boot (if installed)

# Get path to the directory of this file, no matter where it is sourced from
MYDIR=$(dirname ${BASH_SOURCE[0]})

# Setup the Safe2Ditch environment
shopt -s expand_aliases
source "$MYDIR/s2denv"

############################################
# Attempt to get the test name for the bag #
############################################

# Start with an empty bagname
BAGNAME_FILE="$S2DENV/testname"
BAGNAME=
if [ -f "$BAGNAME_FILE" ];
then
	BAGNAME="$(cat $BAGNAME_FILE).bag"

	# If this test already exists, append the current date
	if [ -f "$S2DBAGS/$BAGNAME" ];
	then
		BAGNAME="$(cat BAGNAME_FILE)-`date '+%Y-%m-%d_%H:%M:%S'`.bag"
	fi
else
	BAGNUM_FILE="$S2DENV/bagnum"

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
roslaunch nasa_s2d hardware.launch &

# Record a bag
sleep 5
bagrecord -O "$S2DBAGS/$BAGNAME"
