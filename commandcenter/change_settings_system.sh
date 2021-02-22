#!/usr/bin/env bash
set -e

echo "Changing xml settings $1"
count=0
until (( count++ >= 5 )) || rsync -ze "ssh -o StrictHostKeyChecking=no" pats.tmp $1:code/pats/base/xml/pats_deploy.xml
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
if [ "$#" -ne 1 ]; then
	until (( count++ >= 5 )) || rsync -z drone.tmp $1:code/pats/base/xml/$2
	do
	  echo "$1 retry $count"
	  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
	  sleep 1
	done
	until (( count++ >= 5 )) || rsync -z flightplan.tmp $1:code/pats/base/xml/flightplans/$3
	do
	  echo "$1 retry $count"
	  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
	  sleep 1
	done
fi

if (( count++ < 5 ))
then
 ./restart_system.sh $1
fi
