#!/bin/bash

TRIGGER_DELAY=360
FILE=/home/pats/pats/flags/BenchmarkEntry.txt

while true; do
    if test -f "$FILE"; then
        echo "Benchmark running."
    else
        bash benchmark_short.sh
    fi
    sleep $TRIGGER_DELAY
done

