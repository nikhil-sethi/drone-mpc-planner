#! /usr/bin/env bash

# add /home/pats/code/pats/daemon.sh to /etc/rc.local for autostart

set -e

#/home/pats/code/pats/autoreboot.py &
/home/pats/code/pats/tunnel.sh &
/home/pats/code/pats/tunnel2.sh &
/home/pats/code/pats/autostart.sh >>/home/pats/pats_daemon.log 2>&1 &
