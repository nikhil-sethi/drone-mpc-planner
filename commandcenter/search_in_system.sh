#!/usr/bin/env bash

echo "System: $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	cd ~/pats/data/
	grep -irl "$2" ./$3*/terminal.log
EOF
do
  sleep 1
done
if [ "${count}" -eq 6 ]; then
	echo "Could not reach system :("
fi
echo "................."
