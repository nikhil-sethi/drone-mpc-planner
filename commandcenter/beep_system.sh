#!/usr/bin/env bash
set -ex

ssh -T $1 << EOF
	touch beep_now
EOF