#!/bin/bash

# Pass in lat lon of starting point (origin in Gazebo)
# with syntax: 40.267987,-111.635558 (rock canyon park)
LATLON=$1

# Catch CTRL+C and do SITL cleanup.
trap '{ echo "Cleaning up copter_sitl.sh"; killall xterm && killall mavproxy.py; }' INT

## This is no longer necessary since mavproxy is just for comms relay.
## However, this mechanism could be used for additional MAVProxy initialization
# # Navigate to the aircraft directory where
# # all of the configuration files can be found.
# cd $S2DDSS/$S2DAIRCRAFT

# simulation setup command
SIM_CMD="sim_vehicle.py -v ArduCopter -f gazebo-iris -l $LATLON,0,0 --no-mavproxy"

# MAVProxy command (just for sitl)
# Tell MAVProxy where the aircraft directory lives (--state-basedir) and what the aircraft is called (--aircraft)
MP_CMD="mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 "
MP_CMD+="--map --console --state-basedir=$S2DDSS --aircraft=$S2DAIRCRAFT --cmd='set streamrate 20'"

# Combine the above commands, making SIM_CMD go to the bg and waiting before running the MP_CMD
XTERM_CMD="$SIM_CMD & sleep 5 && $MP_CMD"

# Run these commands in a new window
xterm -fa monospace -fs 12 -n CopterSITL -T CopterSITL -hold -geometry 100x20 -e "$XTERM_CMD"