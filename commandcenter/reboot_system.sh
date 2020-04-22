#!/usr/bin/env bash
set -e
echo "Reboot $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	killall pats || true
	sleep 3
	killall -9 pats || true
	sudo rtcwake -m off -s 120
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done