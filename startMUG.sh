#!/usr/bin/env bash

COUNTER=1
while [  $COUNTER -lt 1000 ]; do
	OUTDIR=/home/slamdunk/kevin/data/$COUNTER
	if [ ! -d "$OUTDIR" ]; then
		echo "Making new data dir: $OUTDIR" 
		COUNTER=1001             
	fi
	let COUNTER=COUNTER+1 
done
sudo mkdir -p $OUTDIR