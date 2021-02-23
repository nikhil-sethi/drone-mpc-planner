#!/usr/bin/env bash
set -e

echo "Replay insect $1"
count=0
until (( count++ >= 5 )) || rsync -ze "ssh -o StrictHostKeyChecking=no" ../base/replay_insects/66-90fps.csv $1:pats/flags/insect_demo
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
