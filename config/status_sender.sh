#!/usr/bin/env bash
set -ex

LOCAL_STATUS_TXT_FILE="/home/$USER/pats_status.txt"
LOCAL_STATUS_IM_FILE="~/home/$USER/pats_monitor_tmp.jpg"
REMOTE_STATUS_TXT_FILE="status/${HOSTNAME}_status.txt"
REMOTE_STATUS_IM_FILE="status/${HOSTNAME}_status.jpg"

while inotifywait -e modify,create, $LOCAL_STATUS_TXT_FILE; do
	rsync -z $LOCAL_STATUS_TXT_FILE mavlab-gpu:$REMOTE_STATUS_TXT_FILE
	if test -f "$LOCAL_STATUS_IM_FILE"; then
		rsync -z $LOCAL_STATUS_IM_FILE mavlab-gpu:$REMOTE_STATUS_IM_FILE
	fi
done