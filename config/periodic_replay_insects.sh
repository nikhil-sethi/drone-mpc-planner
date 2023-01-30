#!/bin/bash

TRIGGER_DELAY=60

while true; do
	cp ~/code/pats/base/replay_insects/001.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/002.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/003.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/004.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/005.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/006.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/007.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
	cp ~/code/pats/base/replay_insects/008.csv  ~/pats/flags/insect_demo
	sleep $TRIGGER_DELAY
done
