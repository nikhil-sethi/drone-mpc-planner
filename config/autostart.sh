#!/usr/bin/env bash

#sleep 20 && usb_modeswitch -v 12d1 -p 1f01 -M 55534243123456780000000000000a11062000000000000100000000000000 &


set -ex
cd /home/pats/code/pats/pc/build/
#export LRS_LOG_LEVEL="DEBUG"

#perform a one time hardware reset (fixes some issues with cold boot and plugged realsense)
./pats rs_reset | /usr/bin/tee terminal.log || true
sleep 3s

while [ 1 ]; do
        dt=$(date '+%d/%m/%Y %H:%M:%S');
        echo "$dt"
		COUNTER=1
		while [  $COUNTER -lt 1000 ]; do
			printf -v PADDEDCOUNTER "%05d" $COUNTER
			OUTDIR=/home/pats/data/$PADDEDCOUNTER
			if [ ! -d "$OUTDIR" ]; then
				echo "Making new data dir: $OUTDIR" 
				COUNTER=10001             
			fi
			let COUNTER=COUNTER+1 
		done
		echo Moving old data to $OUTDIR
		/bin/mkdir -p $OUTDIR
		/bin/mv terminal.log $OUTDIR || true
		/bin/mv logging $OUTDIR || true

	echo "$dt" > terminal.log
	echo "sha:" >> terminal.log
	git rev-parse HEAD >> terminal.log || true
	./pats 2>&1 | /usr/bin/tee --append terminal.log || true

	sleep 10s
done