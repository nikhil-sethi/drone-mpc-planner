#!/usr/bin/env bash

#sleep 20 && usb_modeswitch -v 12d1 -p 1f01 -M 55534243123456780000000000000a11062000000000000100000000000000 &


set -ex
cd /home/pats/code/pats/base/build/

STAT_FN=~/pats_system_info.txt
#export LRS_LOG_LEVEL="DEBUG"

#perform a one time hardware reset (fixes some issues with cold boot and plugged realsense)
./pats --rs-reset | /usr/bin/tee terminal.log || true
sleep 15s # wait some time to enumerate device again after a reset

HOST_ID=$( hostname | tr -dc '0-9' )
DRONE_ID=$(( $HOST_ID ))

while [ 1 ]; do

	while [ -f /home/pats/disable_pats_bkg ]; do
		sleep 10
		echo "Waiting until disable_pats_bkg disappears"
	done

    dt=$(date '+%d/%m/%Y %H:%M:%S');
    echo "$dt"
    fdt=$(date '+%Y%m%d_%H%M%S');
	OUTDIR_LOG=/home/pats/data/$fdt
	OUTDIR_IMAGES=/home/pats/data_images/
	echo Moving daytime monitoring images to $OUTDIR_IMAGES
	/bin/mkdir -p $OUTDIR_IMAGES
	/bin/mv rgb*.png $OUTDIR_IMAGES || true
	/bin/mv stereo*.png $OUTDIR_IMAGES || true
	echo Moving old data to $OUTDIR_LOG
	/bin/mkdir -p $OUTDIR_LOG
	/bin/mv terminal.log $OUTDIR_LOG || true
	/bin/mv logging $OUTDIR_LOG || true

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

	./pats --pats-xml /home/pats/code/pats/base/xml/pats_deploy.xml --drone-id $DRONE_ID 2>&1 | /usr/bin/tee --append terminal.log || true
	../../config/cut_moths.py -i /home/pats/code/pats/base/build/

	sleep 5s
done
