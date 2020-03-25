#!/usr/bin/env bash
set -e

echo "Changing xml settings $1"
count=0
until (( count++ >= 5 )) || rsync -z pats.tmp $1:code/pats/xml/pats_deploy.xml
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
until (( count++ >= 5 )) || rsync -z drone.tmp $1:code/pats/xml/$2
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
until (( count++ >= 5 )) || rsync -z flightplan.tmp $1:code/pats/xml/flightplans/$3
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done

if (( count++ < 5 ))
then
 ./restart_system.sh $1
fi
