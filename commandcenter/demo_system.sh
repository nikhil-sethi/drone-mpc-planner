#!/usr/bin/env bash
set -e

echo "Demo $1"
count=0
until (( count++ >= 5 )) || rsync -ze "ssh -o StrictHostKeyChecking=no" $2 $1:pats_demo.xml
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done