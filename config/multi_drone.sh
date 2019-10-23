#!/usr/bin/env bash


set -ex
cd /home/pats/code/pats/pc/build/


for i in {0..3}
do
   
    dt=$(date '+%d/%m/%Y %H:%M:%S');
    echo "$dt"

	echo "$dt" > terminal.log
	./pats $i 2>&1 | /usr/bin/tee terminal.log || true

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

	echo "Restarting pats with drone id $i +1!"
	sleep 3 #this delay is needed to timeout the multimodule, so that it will re-ask for the drone id
done
