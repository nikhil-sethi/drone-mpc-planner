#!/usr/bin/env bash
set -ex
ssh -T $1 << EOF
	rm ~/data/0* -rf
EOF