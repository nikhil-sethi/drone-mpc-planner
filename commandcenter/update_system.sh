#!/usr/bin/env bash
set -e

echo "Update $1"
count=0
until (( count++ >= 5 )) || ssh -T $1 << EOF
 killall pats || true
 cd code/pats/pc/build
 git fetch
 git reset --hard
 git checkout deploy
 git pr
 cmake ..
 make -j4
 killall -9 pats || true
 echo done
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done