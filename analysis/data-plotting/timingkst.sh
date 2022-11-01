#!/bin/bash

if [ "$1" == "" ]
then
	kst2 -F ~/code/pats/base/build-vscode/logging/timing.csv ~/code/pats/analysis/data-plotting/timing.kst
elif [ "$1" == "-r" ]
then
	kst2 -F ~/code/pats/base/build-vscode/logging/replay/timing.csv ~/code/pats/analysis/data-plotting/timing.kst
else
	kst2 -F $1/timing.csv ~/code/pats/analysis/data-plotting/timing.kst
fi
