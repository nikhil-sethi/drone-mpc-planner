#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	killall pats
	sleep(3)
	killall -9 pats
	sudo rtcwake -m off -s 120
EOF