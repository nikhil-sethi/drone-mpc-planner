#!/bin/sh

if [ "$1" != "" ]
then
	kst2 -F $1/log.csv ~/code/pats/analysis/data-plotting/control.kst
else
	kst2 -F ~/code/pats/base/build-vscode/logging/log.csv ~/code/pats/analysis/data-plotting/control.kst 
fi
