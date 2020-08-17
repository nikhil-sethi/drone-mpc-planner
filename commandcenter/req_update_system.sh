#!/usr/bin/env bash
set -e
echo "Requesting update $1"
count=0
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	touch pats_cc_update_request
EOF
do
  echo "Retry $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done
