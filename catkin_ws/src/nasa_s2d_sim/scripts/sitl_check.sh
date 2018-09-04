#!/bin/bash

PIPE=$1

# make a temporary named pipe that will allow us to communicate with the SITL process
if [[ ! -p $PIPE ]]; then
    mkfifo "$PIPE"
fi

# clean up after ourselves on exit -- we're not animals!
trap  "rm -f $PIPE" EXIT

# block until we see the magic words
while true
do
    if read LINE;
    then
        # Note that this is a line that starts with the literal string
        if [[ $LINE = 'RiTW: Starting ArduCopter'* ]];
        then
            break;
        fi
    fi
done < $PIPE