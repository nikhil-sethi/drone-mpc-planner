#! /usr/bin/env bash

set -e

/home/slamdunk/kevin/mu-g/autoreboot.py &
/home/slamdunk/kevin/mu-g/tunnel.sh &
/home/slamdunk/kevin/mu-g/tunnel2.sh &
/home/slamdunk/kevin/mu-g/autostart.sh >>/home/slamdunk/kevin/pats_daemon.log 2>&1 &
