#!/usr/bin/env bash
set -ex

rsync -z pats.tmp $1:code/pats/xml/pats_deploy.xml
rsync -z drone.tmp $1:code/pats/xml/$2
rsync -z flightplan.tmp $1:code/pats/xml/flightplans/$3

./restart_system.sh $1