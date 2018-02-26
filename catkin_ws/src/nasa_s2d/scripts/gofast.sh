#!/bin/bash

# Wait for 5 seconds before doing this stuff (to let mavros settle)
sleep 15

# Let the world know what you are doing
set -x

rosrun mavros mavsys rate --all 0
rosrun mavros mavsys rate --extra1 50
rosrun mavros mavsys rate --ext-status 25
rosrun mavros mavsys rate --position 50
rosrun mavros mavsys rate --raw-sensors 50

# Back to silent
set +x
