#!/bin/bash
#
#
#

LOGNAME="mavproxy_$S2DTESTNAME.log"
LOG="$S2DBAGS/$LOGNAME"
QUIET=false

echo -e "`date`" >> $LOG
echo -e "$S2DTESTNAME" >> $LOG

## This is no longer necessary since mavproxy is just for comms relay.
## However, this mechanism could be used for additional MAVProxy initialization
# # Navigate to the aircraft directory where
# # all of the configuration files can be found.
# cd $S2DDSS/$S2DAIRCRAFT

# add or remove the tones based on the QUIET variable
if [ "$QUIET" = true ];
then
	TONE1=
	TONE2=
else
	TONE1="playtune A"
	TONE2="playtune MFT90O2C16C16C16F8.A8C16C16C16F8.A4P16P8"
fi

mavproxy.py --mav20 --master=/dev/ttyTHS2 --baudrate=921600 --cmd="set requireexit True; ${TONE1}; exit"

mavproxy.py --mav20 --master=/dev/ttyTHS2 --baudrate=921600 --moddebug=3 --source-system=250 --out=127.0.0.1:14550 --cmd="set streamrate 20; ${TONE2};" --daemon >> $LOG