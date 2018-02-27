#!/bin/bash
#
#
#

echo -e "\n\n\n" >> ~/mavproxy.log
echo -e "`date`" >> ~/mavproxy.log

mavproxy.py --master=/dev/ttyTHS2 --baudrate=921600 --moddebug=3 --source-system=250 --out=127.0.0.1:14550 --cmd="set streamrate 100; module load safe2ditch" >> ~/mavproxy.log
