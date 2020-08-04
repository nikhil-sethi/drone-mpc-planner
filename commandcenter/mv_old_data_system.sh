#!/usr/bin/env bash
set -e
until (( count++ >= 5 )) || ssh -o StrictHostKeyChecking=no -T $1 << EOF
	cd ~/data
	mkdir -p ../tmp_bla
	mv 20200803* ../tmp_bla/
	mv 20200804* ../tmp_bla/

	mkdir -p ../data_old
	echo * | xargs mv -t ../data_old

	mv ../tmp_bla/* ./
	rm ../tmp_bla -r
EOF
do
  echo "Retry $1 $count"
  paplay /usr/share/sounds/ubuntu/stereo/bell.ogg
  sleep 1
done