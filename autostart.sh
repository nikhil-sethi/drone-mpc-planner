#!/usr/bin/env bash

set -e

while [ 1 ]; do
	echo "Starting pats in 20s..."

	sleep 20s

	COUNTER=1
	while [  $COUNTER -lt 1000 ]; do
		OUTDIR=/home/pats/data/$COUNTER
		if [ ! -d "$OUTDIR" ]; then
			echo "Making new data dir: $OUTDIR" 
			COUNTER=1001             
		fi
		let COUNTER=COUNTER+1 
	done
	sudo mkdir -p $OUTDIR
	

	sudo /home/pats/code/pats/pc/build/pats $OUTDIR > >( sudo tee -a  $OUTDIR/stdlog.txt) 2> >(sudo tee -a $OUTDIR/errorlog.txt >&2) || true 
	#sudo /home/slamdunk/hv/share/mu-g/SLAMdunk/build/MUG $OUTDIR /factory . > >( sudo tee -a  $OUTDIR/stdlog.txt) 2> >(sudo tee -a $OUTDIR/errorlog.txt >&2) || true 

	sleep 1s
done
