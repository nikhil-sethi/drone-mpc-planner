#!/usr/bin/env bash
set -e
echo "Reboot $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	killall pats || true
	sleep 3
	killall -9 pats || true
	if [ -c /dev/baseboard ]
	then
		sudo rtcwake -m no -s 120
		sudo systemctl poweroff
	else
		sudo rtcwake -m no -s 120
		sudo swapoff -a
		sudo systemctl poweroff -f
	fi
EOF
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done