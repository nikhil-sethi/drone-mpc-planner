#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
 cd code/pats/pc/build
 git reset --hard
 git checkout deploy
 git pr
 cmake ..
 make -j4
 killall -9 pats
 echo done
EOF