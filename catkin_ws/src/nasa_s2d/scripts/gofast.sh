#!/bin/bash

# Wait for 5 seconds before doing this stuff (to let mavros settle)
sleep 5

# Let the world know what you are doing
set -x

rosrun mavros mavsys rate --extra1 20
rosrun mavros mavsys rate --ext-status 10
rosrun mavros mavsys rate --position 20
rosrun mavros mavsys rate --raw-sensors 20

# Back to silent
set +x
