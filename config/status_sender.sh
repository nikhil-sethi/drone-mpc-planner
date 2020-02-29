#!/usr/bin/env bash
set -x

LOCAL_STATUS_TXT_FILE="/home/$USER/pats_status.txt"
LOCAL_SYSTEM_TXT_FILE="/home/$USER/pats_system_info.txt"
LOCAL_STATUS_IM_FILE="/home/$USER/pats_monitor_tmp.jpg"
REMOTE_STATUS_TXT_FILE="status/${HOSTNAME}_status.txt"
REMOTE_SYSTEM_TXT_FILE="status/${HOSTNAME}_system.txt"
REMOTE_STATUS_IM_FILE="status/${HOSTNAME}_status.jpg"

while [ ! -f "$LOCAL_STATUS_TXT_FILE" ]; do
	sleep 10
	echo "Warning: $LOCAL_STATUS_TXT_FILE does not exist"
done
while inotifywait -e modify,create, $LOCAL_STATUS_TXT_FILE; do
	if test -f "$LOCAL_STATUS_TXT_FILE"; then
		rsync -z $LOCAL_STATUS_TXT_FILE mavlab-gpu:$REMOTE_STATUS_TXT_FILE
	fi
	if test -f "$LOCAL_SYSTEM_TXT_FILE"; then
		rsync -z $LOCAL_SYSTEM_TXT_FILE mavlab-gpu:$REMOTE_SYSTEM_TXT_FILE
	fi
	if test -f "$LOCAL_STATUS_IM_FILE"; then
		rsync -z $LOCAL_STATUS_IM_FILE mavlab-gpu:$REMOTE_STATUS_IM_FILE
	fi
done