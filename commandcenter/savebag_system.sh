#!/usr/bin/env bash
set -e
t="$(date +"%Y%m%d%H%M%S")"
echo "Save bag $1"
count=0
until (( count++ >= 5 )) || ssh -T $1 << EOF
	killall pats || true
	sleep 3
	mv ~/code/pats/pc/build/logging ~/data/bag_$t
	killall pats -9 || true
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done