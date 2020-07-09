#!/usr/bin/env bash
set -e

echo "System: $1"
ssh -o StrictHostKeyChecking=no -T $1 << EOF
	cd ~/data/
	grep -irl "$2" ./20200708_*
EOF
echo "................."
