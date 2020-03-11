#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
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