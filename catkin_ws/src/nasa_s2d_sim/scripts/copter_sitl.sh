#!/bin/bash

# Pass in the name of the aircraft to use.
# This is simply for grouping simulation files
SITL_LOCATION=$1
AIRCRAFT="sitl_${SITL_LOCATION// /}" # remove spaces

# Pass in lat lon of starting point (origin in Gazebo)
# with syntax: 40.267987,-111.635558 (rock canyon park)
LATLON=$2

# Should I be quiet? This is useful for headless mode when running sims
if [[ $3 ]];
then
    QUIET=$3
fi

# Catch CTRL+C and do SITL cleanup.
trap '{ echo "Cleaning up copter_sitl.sh"; killall xterm && killall -9 mavproxy.py; }' INT

# Directory to call MAVProxy from (where eeprom.bin will be, i.e., waypoints)
# and where the AIRCRAFT directory with SITL flight logs will be
LAUNCH_DIR=$HOME/.ros/

# Navigate to the launch directory where the eeprom.bin will be stored
cd $LAUNCH_DIR/$AIRCRAFT

# simulation setup command
SIM_CMD="sim_vehicle.py -v ArduCopter -f gazebo-iris -l $LATLON,0,0 --no-mavproxy"

# MAVProxy command (just for sitl)
# Tell MAVProxy where the aircraft directory lives (--state-basedir) and what the aircraft is called (--aircraft)
MP_CMD="mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 "
MP_CMD+="--state-basedir=$LAUNCH_DIR --aircraft=$AIRCRAFT --cmd='set streamrate 20'"

# Combine the above commands, making SIM_CMD go to the bg and waiting before running the MP_CMD
XTERM_CMD="$SIM_CMD & sleep 5 && $MP_CMD"

if [[ $QUIET ]];
then
    eval $XTERM_CMD
else
    # Run these commands in a new window
    xterm -fa monospace -fs 12 -n CopterSITL -T CopterSITL -hold -geometry 100x20 -e "$XTERM_CMD --map --console"
fi