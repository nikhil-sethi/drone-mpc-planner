#!/usr/bin/env bash
set -e
echo "Beep $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	touch beep_now
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done