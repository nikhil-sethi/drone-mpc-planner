#!/usr/bin/env bash
set -x

LOCAL_STATUS_TXT_FILE="/home/$USER/pats_status.txt"
LOCAL_SYSTEM_TXT_FILE="/home/$USER/pats_system_info.txt"
LOCAL_STATUS_IM_FILE="/home/$USER/pats_monitor_tmp.jpg"
LOCAL_PATS_XML="/home/$USER/code/pats/xml/"
REMOTE_STATUS_TXT_FILE="status/${HOSTNAME}/status.txt"
REMOTE_SYSTEM_TXT_FILE="status/${HOSTNAME}/system.txt"
REMOTE_STATUS_IM_FILE="status/${HOSTNAME}/status.jpg"
REMOTE_PATS_XML="status/${HOSTNAME}/"

while [ ! -f "$LOCAL_STATUS_TXT_FILE" ]; do
	sleep 10
	echo "Warning: $LOCAL_STATUS_TXT_FILE does not exist"
done


inotifywait -q -m -r -e modify -e create -e delete $LOCAL_STATUS_TXT_FILE | \
while read event; do

	if test -d "$LOCAL_PATS_XML"; then
		rsync -az $LOCAL_PATS_XML mavlab-gpu:$REMOTE_PATS_XML
	fi
	if test -f "$LOCAL_STATUS_TXT_FILE"; then
		rsync -z $LOCAL_STATUS_TXT_FILE mavlab-gpu:$REMOTE_STATUS_TXT_FILE
	fi
	if test -f "$LOCAL_SYSTEM_TXT_FILE"; then
		rsync -z $LOCAL_SYSTEM_TXT_FILE mavlab-gpu:$REMOTE_SYSTEM_TXT_FILE
	fi
	if test -f "$LOCAL_STATUS_IM_FILE"; then
		rsync -z $LOCAL_STATUS_IM_FILE mavlab-gpu:$REMOTE_STATUS_IM_FILE
	fi
	while [ -f /home/pats/disable_pats_bkg ]; do
		sleep 10
		echo "Waiting until disable_pats_bkg disappears"
	done
done