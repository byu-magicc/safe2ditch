#!/bin/bash

# Pass in lat lon of starting point (origin in Gazebo)
# with syntax: 40.267987,-111.635558 (rock canyon park)
LATLON=$1

# Catch CTRL+C and do SITL cleanup.
trap '{ echo "Cleaning up copter_sitl.sh"; killall xterm && killall mavproxy.py; }' INT

# Navigate to the aircraft directory where
# all of the configuration files can be found.
cd $S2DDSS/$S2DAIRCRAFT

# Tell MAVProxy where the aircraft directory lives (--state-basedir)
# and what the aircraft is called (--aircraft)
CMD="sim_vehicle.py -v ArduCopter -f gazebo-iris -l $LATLON,0,0 --map --console -m --state-basedir=$S2DDSS --aircraft=$S2DAIRCRAFT"
xterm -fa monospace -fs 12 -n CopterSITL -T CopterSITL -hold -geometry 100x20 -e $CMD