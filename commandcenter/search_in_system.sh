#!/usr/bin/env bash

echo "System: $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	cd ~/data/
	grep -irl "$2" ./$3*/terminal.log > vids.txt
	cat vids.txt
	mkdir -p "$1"
	python3 ~/code/pats/commandcenter/moth_cutter_search.py -i ~/data/ -o ~/data/"$1" -t vids.txt
	
EOF
do
  sleep 1
done
if [ "${count}" -eq 6 ]; then
	echo "Could not reach system :("
fi
echo "................."
