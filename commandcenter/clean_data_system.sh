#!/usr/bin/env bash
set -e
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	rm ~/data/0* -rf
	rm ~/data/bag* -rf
EOF
do
  echo "Retry $1 $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done