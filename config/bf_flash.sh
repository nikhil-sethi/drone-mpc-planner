#!/usr/bin/env bash
set -e
i=1
sp="/-\|"
port="/dev/ttyACM0"
echo "Checking port $port ..."
echo -n ' '

FIRMWARE=./trashcan_firmware.bin
if [ $# -gt 0 ]; then
    FIRMWARE=$1
fi

while [ ! -c $port ]
do
	sleep 0.1
	printf "\b${sp:i++%${#sp}:1}"
done
printf "\b \bOK\n"
echo Uploading $FIRMWARE...
sleep 0.5
./bf_to_dfu.py
sleep 1
dfu-util -s 0x08000000 -a 0 -R -D $FIRMWARE

while [ ! -c $port ]
do
	sleep 0.1
	printf "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b${sp:i%${#sp}:1}Human, replug usb NOW${sp:i++%${#sp}:1}"
done
printf "\n"
sleep 1
./bf_upload_settings.py