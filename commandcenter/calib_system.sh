#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	touch calib_now
EOF