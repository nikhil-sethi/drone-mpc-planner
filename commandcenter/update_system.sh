#!/usr/bin/env bash
set -e

echo "Update $1"
count=0
branch=$(git rev-parse --abbrev-ref HEAD)
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
 killall pats || true
 cd code/pats/pc/build
 git fetch
 git reset --hard
 git checkout $branch
 git pr
 cmake -DCMAKE_BUILD_TYPE=RELEASE ..
 make -j4
 killall -9 pats || true
 echo done
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done