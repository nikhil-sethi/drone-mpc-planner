#! /usr/bin/env bash
set -e

# add /home/pats/code/pats/daemon.sh to /etc/rc.local for autostart
# this script creates a screen session that runs as non-elevated user pats
# a seperate screen window is added for each process
# use screen -r to re-attach to below screen session later, ctrl a,  <number> to go to the <number> window
user=pats

/bin/su - $user -c "/usr/bin/screen -dm -S daemon ~/code/pats/dashboard/watch_config/screens.sh"
