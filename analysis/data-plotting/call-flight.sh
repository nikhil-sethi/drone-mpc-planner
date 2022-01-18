#!/bin/sh

if [ "$1" != "" ]
then
	kst2 -F $1/log_flight$2.csv ~/code/pats/analysis/data-plotting/flight.kst
else
	kst2 -F ~/code/pats/base/build-vscode/logging/log_flight1.csv ~/code/pats/analysis/data-plotting/flight.kst
fi
