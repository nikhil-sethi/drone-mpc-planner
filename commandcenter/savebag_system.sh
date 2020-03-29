#!/usr/bin/env bash
set -e
t="$(date +"%Y%m%d%H%M%S")"
echo "Save bag $1"
count=0
until (( count++ >= 5 )) || ssh -T $1 << EOF
	if pgrep -x "pats" > /dev/null
	then
		killall pats || true
	fi
	sleep 0.5
	mv ~/code/pats/pc/build/logging ~/data/bag_$t
	mv ~/code/pats/pc/build/terminal.log ~/data/bag_$t
	sleep 2
	if pgrep -x "pats" > /dev/null
	then
		killall pats -9 || true
	fi
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done