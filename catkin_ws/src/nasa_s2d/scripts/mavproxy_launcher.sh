#!/bin/bash
#
#
#

echo -e "\n\n\n" >> ~/mavproxy.log
echo -e "`date`" >> ~/mavproxy.log

# MFT90O2C16C16C16F8.A8C16C16C16F8.A4P16P8

mavproxy.py --mav20 --master=/dev/ttyTHS2 --baudrate=921600 --moddebug=3 --source-system=250 --out=127.0.0.1:14550 --cmd="set streamrate 20; module load safe2ditch; playtune MFT90O2C16C16C16F8.A8C16C16C16F8.A4P16P8" --daemon >> ~/mavproxy.log
