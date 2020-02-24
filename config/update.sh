#!/usr/bin/env bash
set -ex

ssh -t $1 << EOF
 cd code/pats/pc/build
 git pr
 cmake ..
 make -j4
 killall -9 pats
 echo done
EOF