#!/usr/bin/env bash
set -e

#usage: ./search_all_results.sh 11 13 "best_interception_distance:0" 20200919

for i in $(seq $1 $2); do
	echo "pats${i}"
	./search_in_results.sh "pats${i}" $3 $4 > pats${i}.search.tmp &
done

repeat=1
while [[ $repeat -eq 1 ]]; do
	repeat=0
	sleep 0.5
	tmp=1
	echo -n "Waiting for: "
	for i in $(seq $1 $2); do
		NUM_LINES=$(grep -ir "................." ./pats$i.search.tmp | wc -l)
		# echo "pats$i: $NUM_LINES"
		if [ "${NUM_LINES}" -eq 0 ]; then
			echo -n "$i "
			repeat=1
		fi
	done
	echo
done

echo
echo "Results!"
echo

for i in $(seq $1 $2); do
	cat ./pats$i.search.tmp
done