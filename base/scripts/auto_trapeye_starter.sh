#!/usr/bin/env bash

for i in $(seq 1 $1)
do
	echo $i $1
	~/pats/release/build/trapeye &
done

while true
do
	sleep 1000
done
