#!/usr/bin/env bash

echo "START ~/pats/release/build/trapeye $1 times"
while true
do
	dt=$(date '+%d/%m/%Y %H:%M:%S');
	echo "$dt"

	for i in $(seq 1 $1)
	do
		echo $i $1
		DIR=$HOME/trapeye/logs/process_$i.log
		~/pats/release/build/trapeye  2>&1 | ts '[%a %m-%d %H:%M]'  2>&1 | tee -a $DIR &
	done

	~/pats/release/scripts/endless_wait.sh # this is a seperate process, so that we can kill it as to trigger a restart of the trapeye processes
	killall trapeye
	sleep 1
done
