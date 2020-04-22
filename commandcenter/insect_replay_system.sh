#!/usr/bin/env bash
set -e

echo "Replay insect $1"
count=0
until rsync -ze "ssh -o StrictHostKeyChecking=no" ../pc/insect_logs/56.csv $1:insect_demo
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done