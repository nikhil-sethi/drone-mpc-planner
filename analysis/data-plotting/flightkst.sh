#!/bin/bash

flight_kst="${HOME}/code/pats/analysis/data-plotting/flight.kst"
alternative_flight_kst="${HOME}/code/kst/flight.kst"
if [ -f ${alternative_flight_kst} ]; then
	echo "Use custome kst-config.."
	flight_kst=$alternative_flight_kst
fi

if [ "$1" == "" ]
then
	kst2 -F ~/code/pats/base/build-vscode/logging/log_flight1.csv $flight_kst
elif [ "$1" == "-r" ]
then
	kst2 -F ~/code/pats/base/build-vscode/logging/replay/log_flight$2.csv $flight_kst
else
	kst2 -F $1/log_flight$2.csv $flight_kst
fi
