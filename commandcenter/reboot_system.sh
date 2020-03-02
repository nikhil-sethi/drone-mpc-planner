#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	sudo rtcwake -m off -s 20
EOF