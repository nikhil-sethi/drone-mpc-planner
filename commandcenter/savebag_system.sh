#!/usr/bin/env bash
set -ex
t="$(date +"%Y%m%d%H%M%S")"
ssh -T $1 << EOF
	killall pats || true
	sleep 3
	mv ~/code/pats/pc/build/logging ~/data/bag_$t
	killall pats -9 || true
EOF