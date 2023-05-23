#!/usr/bin/env bash

while true
do
	dt=$(date '+%d/%m/%Y %H:%M:%S');
	echo "$dt"

	for i in $(seq 1 $1)
	do
		echo $i $1
		DIR=$HOME/trapeye/logs/process_$i.log
		~/pats/release/build/trapeye | ts '[%a %H:%M]' | tee -a $DIR &
	done

	~/pats/release/scripts/endless_wait.sh # this is a seperate process, so that we can kill it as to trigger a restart of the trapeye processes
	killall trapeye
	sleep 1
done
