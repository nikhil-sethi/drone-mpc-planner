#!/usr/bin/env bash

echo "System: $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	cd ~/pats/data/
	grep -ir "$2" ./$3*/logging/results.txt
EOF
do
  sleep 1
done
if [ "${count}" -eq 6 ]; then
	echo "Could not reach system :("
fi
echo "................."


#grep -ir "best_interception_distance:0" ./20200919*