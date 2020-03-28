#!/usr/bin/env bash
set -e
i=1
sp="/-\|"
port="/dev/ttyACM0"
echo "Checking port $port ..."
echo -n ' '
while [ ! -c $port ]
do
	sleep 0.1
	printf "\b${sp:i++%${#sp}:1}"
done
printf "\b \bOK"
sleep 0.5
./bf_to_dfu.py
sleep 1
dfu-util -s 0x08000000 -a 0 -R -D ~/code/betaflight/obj/betaflight_4.0.6_CRAZYBEEF4FR.bin

while [ ! -c $port ]
do
	sleep 0.1
	printf "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b${sp:i%${#sp}:1}Human, replug usb NOW${sp:i++%${#sp}:1}"
done
printf "\n"
sleep 1
./bf_upload_settings.py