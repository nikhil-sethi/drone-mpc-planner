#!/usr/bin/env bash

set -ex
cd /home/pats/code/pats/base/build/

mkdir -p /home/pats/pats/xml
mkdir -p /home/pats/pats/flags
mkdir -p /home/pats/pats/status
mkdir -p /home/pats/pats/logs
mkdir -p /home/pats/pats/jsons
mkdir -p /home/pats/pats/data
mkdir -p /home/pats/pats/images

STAT_FN=~/pats/status/system_info.txt
#export LRS_LOG_LEVEL="DEBUG"

HOST_ID=$( hostname | tr -dc '0-9' )
DRONE_ID=$(( $HOST_ID ))

#perform a one time hardware reset (fixes some issues with cold boot and plugged realsense)
./executor --rs-reset | /usr/bin/tee terminal_cam_reset.log || true
sleep 15s # wait some time to enumerate device again after a reset

while [ 1 ]; do

	while [ -f /home/pats/pats/flags/disable ]; do
		sleep 10
		echo "Waiting until disable flag disappears"
	done

	dt=$(date '+%d/%m/%Y %H:%M:%S');
	echo "$dt"
	fdt=$(date '+%Y%m%d_%H%M%S');
	OUTDIR_LOG=/home/pats/pats/data/$fdt
	OUTDIR_IMAGES=/home/pats/pats/images/
	echo Moving daytime monitoring images to $OUTDIR_IMAGES
	/bin/mkdir -p $OUTDIR_IMAGES
	/bin/mv rgb*.png $OUTDIR_IMAGES || true
	/bin/mv stereo*.png $OUTDIR_IMAGES || true
	echo Moving old data to $OUTDIR_LOG
	/bin/mv logging/ ${OUTDIR_LOG}/ || true
	/bin/mv terminal.log ${OUTDIR_LOG}/ || true

	echo "Hostname: $HOSTNAME" > $STAT_FN
	echo "Drone ID: $DRONE_ID" >> $STAT_FN
	SHA="$(git rev-parse HEAD)"
	echo "sha: $SHA"  >> $STAT_FN
	DF="$(df -h -x squashfs -x tmpfs -x devtmpfs)"
	echo "df: $DF"  >> $STAT_FN
	ip  -o -4 -f inet a show up primary scope global >> $STAT_FN

	echo "$dt" > terminal.log
	echo "sha:$SHA" >> terminal.log
	echo "Hostname: $HOSTNAME" >> terminal.log
	echo "Drone ID: $DRONE_ID" >> terminal.log

	./executor --drone-id $DRONE_ID 2>&1 | /usr/bin/tee --append terminal.log || true

	sleep 5s
done
