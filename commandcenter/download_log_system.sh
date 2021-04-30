#!/usr/bin/env bash
set -e
fdt=$(date '+%Y%m%d_%H%M%S');
OUTDIR_LOG=/home/pats/pats/data/$fdt
echo "Save bag $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	if pgrep -x "pats" > /dev/null
	then
		killall pats || true
	fi
	sleep 0.5
	mkdir ${OUTDIR_LOG}
	mv ~/code/pats/base/build/logging ${OUTDIR_LOG} || true
	mv ~/code/pats/base/build/terminal.log $OUTDIR_LOG/ || true
	touch ${OUTDIR_LOG}/cc_download
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
rsync -avPzhe "ssh -o StrictHostKeyChecking=no" $1:pats/data/$fdt ~/Downloads/pats_data/$1