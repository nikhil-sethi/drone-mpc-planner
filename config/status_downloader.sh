#!/usr/bin/env bash
DEST=~/Downloads/pats_status
SRC=mavlab-gpu:/home/pats/status/
mkdir -p $DEST

while true; do
    rsync -zva --timeout=3 $SRC $DEST
    sleep 1
done
