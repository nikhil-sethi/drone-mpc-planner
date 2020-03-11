#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	killall pats || true
	sleep(3)
	killall -9 pats || true
	sudo rtcwake -m off -s 120
EOF