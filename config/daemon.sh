#! /usr/bin/env bash

# add /home/pats/code/pats/daemon.sh to /etc/rc.local for autostart
# this script creates a screen session that runs as non-elevated user pats
# a seperate screen window is added for each process 
# use screen -r to re-attach to below screen session later, ctrl a,  <number> to go to the <number> window

set -e


su - pats -c "/usr/bin/screen -dmS mama bash -c '/home/pats/code/pats/tunnel.sh; exec bash'"
su - pats -c "/usr/bin/screen -S mama -x -X screen bash -c '/home/pats/code/pats/tunnel2.sh; exec bash'"
su - pats -c "/usr/bin/screen -S mama -x -X screen bash -c '/home/pats/code/pats/autostart.sh 2>&1 | tee -a /home/pats/pats_daemon.log ; exec bash'"


#old non screen commands:
#/home/pats/code/pats/autoreboot.py &
#/home/pats/code/pats/tunnel.sh &
#/home/pats/code/pats/tunnel2.sh &
#/home/pats/code/pats/autostart.sh >>/home/pats/pats_daemon.log 2>&1 &
