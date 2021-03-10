#!/usr/bin/env bash
set -e
echo "Blink $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	touch pats/flags/blink_now
EOF
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done