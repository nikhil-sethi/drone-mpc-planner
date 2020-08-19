#!/usr/bin/env bash

#sleep 20 && usb_modeswitch -v 12d1 -p 1f01 -M 55534243123456780000000000000a11062000000000000100000000000000 &


set -ex
cd /home/pats/code/pats/pc/build/

STAT_FN=~/pats_system_info.txt
#export LRS_LOG_LEVEL="DEBUG"

#perform a one time hardware reset (fixes some issues with cold boot and plugged realsense)
./pats --rs_reset | /usr/bin/tee terminal.log || true
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
	OUTDIR=/home/pats/data/$fdt
	echo Moving old data to $OUTDIR
	/bin/mkdir -p $OUTDIR
	/bin/mv terminal.log $OUTDIR || true
	/bin/mv logging $OUTDIR || true

	echo "Hostname: $HOSTNAME" > $STAT_FN
	echo "Drone ID: $DRONE_ID" >> $STAT_FN
	SHA="$(git rev-parse HEAD)"
	echo "sha: $SHA"  >> $STAT_FN
	DF="$(df -h -x squashfs -x tmpfs -x devtmpfs)"
	echo "df: $DF"  >> $STAT_FN
	ip  -o -4 -f inet a show up primary scope global >> $STAT_FN

	echo "$dt" > terminal.log
	echo "sha:$SHA" >> terminal.log

	./pats --pats-xml /home/pats/code/pats/xml/pats_deploy.xml --drone-id $DRONE_ID 2>&1 | /usr/bin/tee --append terminal.log || true

	sleep 5s
done
