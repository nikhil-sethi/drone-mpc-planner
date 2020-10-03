#!/usr/bin/env bash
set -e
echo "Restarting $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	if pgrep -x "pats" > /dev/null
	then
		killall pats || true
		sleep 2
		if pgrep -x "pats" > /dev/null
		then
			killall pats -9 || true
		fi
	fi
EOF
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
