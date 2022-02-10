#!/usr/bin/env bash
set -e

echo "Update $1"
count=0
branch=$(git rev-parse --abbrev-ref HEAD)
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
 killall executor || true
 cd code/pats/base/build
 rm executor
 git fetch
 git reset --hard
 git checkout $branch
 git pr
 cmake -DCMAKE_BUILD_TYPE=RELEASE ..
 make -j$(nproc)
 killall -9 executor || true
 echo update done
EOF
do
  echo "$1 retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done