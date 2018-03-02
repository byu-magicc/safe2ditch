#!/bin/bash
#
# This script gets run via systemctl on boot (if installed)

# Get path to the directory of this file, no matter where it is sourced from
MYDIR=$(dirname ${BASH_SOURCE[0]})

# Setup the Safe2Ditch environment
shopt -s expand_aliases
source "$MYDIR/s2denv"

# Start the system
roslaunch nasa_s2d hardware.launch &

# Increment the TESTNUM and save to file
TESTNUM=$(cat $S2DENV/testnum)
TESTNUM=$((TESTNUM+1))
echo $TESTNUM > "$S2DENV/testnum"

# Record a bag
sleep 5
bagrecord -O "$S2DBAGS/$S2DTESTNAME.bag"
