#!/usr/bin/env bash


set -ex
cd /home/pats/code/pats/pc/build/


for i in {1..3}
do
   
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
	./pats $i 2>&1 | /usr/bin/tee terminal.log || true

	echo "Starting pats $i +1 in 10s!"
	sleep 10s
done