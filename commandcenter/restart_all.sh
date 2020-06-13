#!/usr/bin/env bash
set -e

for i in $(seq $1 $2); do 
	echo "pats${i}" 
	./restart_system.sh "pats${i}" &
done
