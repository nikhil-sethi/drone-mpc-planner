#!/usr/bin/env bash

set -e

while [ 1 ]; do
	echo "Starting mu-g in 20s..."

	for file in "/sys/class/leds/"{front,rear}":"{left,right}":"{blue,green,red}"/brightness"; do
	    echo 0 > $file
	done

	for file in "/sys/class/leds/"{front,rear}":"{left,right}":blue/brightness"; do
	    echo 255 > $file
	done

	sleep 20s

	startMUG.sh

	for file in "/sys/class/leds/"{front,rear}":"{left,right}":"{blue,green,red}"/brightness"; do
	    echo 0 > $file
	done
	for file in "/sys/class/leds/"{front,rear}":"{left,right}":green/brightness"; do
	    echo 255 > $file
	done

	sudo /home/slamdunk/kevin/mu-g/SLAMdunk/build/MUG $OUTDIR /factory . > >( sudo tee -a  $OUTDIR/stdlog.txt) 2> >(sudo tee -a $OUTDIR/errorlog.txt >&2) || true 
	#sudo /home/slamdunk/hv/share/mu-g/SLAMdunk/build/MUG $OUTDIR /factory . > >( sudo tee -a  $OUTDIR/stdlog.txt) 2> >(sudo tee -a $OUTDIR/errorlog.txt >&2) || true 

	for file in "/sys/class/leds/"{front,rear}":"{left,right}":"{blue,green,red}"/brightness"; do
	    echo 0 > $file
	done
	for file in "/sys/class/leds/"{front,rear}":"{left,right}":"{red,green}"/brightness"; do
	    echo 255 > $file
	done

	sleep 1s
done