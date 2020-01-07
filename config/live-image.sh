#!/usr/bin/env bash
set -ex
sftp $1:pats_monitor_tmp.jpg ~/Desktop/pats-live-image.jpg
eog ~/Desktop/pats-live-image.jpg &
while [ 1 ]; do
	sleep 5
	sftp $1:pats_monitor_tmp.jpg ~/Desktop/pats-live-image.jpg
done