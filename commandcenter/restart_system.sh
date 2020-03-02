#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	killall pats
	sleep 3
	killall pats -9
EOF