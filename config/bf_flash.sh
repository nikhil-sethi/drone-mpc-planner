#!/usr/bin/env bash

set -e
i=1
sp="/-\|"
echo "Checking port..."
echo -n ' '
while [ ! -c /dev/ttyACM0 ]
do
	sleep 0.1
	printf "\b${sp:i++%${#sp}:1}"
done

sleep 1
./bf_to_dfu.py
sleep 1
dfu-util -s 0x08000000 -a 0 -R -D trashcan_firmware.bin

echo Human, replug usb NOW.
while [ ! -c /dev/ttyACM0 ]
do
	sleep 0.1
	printf "\b${sp:i++%${#sp}:1}"
done
sleep 1
./bf_upload_settings.py