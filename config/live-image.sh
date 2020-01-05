#!/usr/bin/env bash
set -ex
while [ 1 ]; do
	sftp pats@$1:~/monitor.jpg ~/Desktop/pats-live-image.jpg
	eog ~/Desktop/pats-live-image.jpg
	sleep(5)
done