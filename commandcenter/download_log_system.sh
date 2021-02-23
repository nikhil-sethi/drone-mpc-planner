#!/usr/bin/env bash
set -e
t="$(date +"%Y%m%d%H%M%S")"
echo "Save bag $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	if pgrep -x "pats" > /dev/null
	then
		killall pats || true
	fi
	sleep 0.5
	mv ~/code/pats/base/build/logging ~/pats/data/dl_$t
	mv ~/code/pats/base/build/terminal.log ~/pats/data/dl_$t
	sleep 2
	if pgrep -x "pats" > /dev/null
	then
		killall pats -9 || true
	fi
EOF
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done

set -xe
mkdir -p ~/Downloads/pats_data/$1
rsync -avPzhe "ssh -o StrictHostKeyChecking=no" $1:pats/data/dl* ~/Downloads/pats_data/$1