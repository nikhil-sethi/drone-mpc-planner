#!/usr/bin/env bash

# uptime -p
# cd ~/code/pats && echo git sha: $(git rev-parse HEAD)
# echo image version: $(cat ~/dependencies/image_version) kernel: $(uname -r) $(lsb_release -d)
# lscpu | grep 'Model name' &&
# tail ~/pats/logs/all_errors.log.20220803 -n +2
if [ -f /var/run/reboot-required ]; then echo 'Reboot required'; fi
if [ -f '/home/pats/pats/xml/pats.xml' ] ; then
    if ( ! grep -q "Version=\"1.20\"" /home/pats/pats/xml/pats.xml ) ; then echo pats.xml VERSION ERROR ; fi
fi
if [ -e '/dev/baseboard' ] && [ -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD AVAILABLE BUT DISABLED' ; fi
if [ ! -e '/dev/baseboard' ] && [ ! -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD NOT DETETECTED AND NOT DISABLED' ; fi
#if [ -e '/dev/pats_mm0' ] ; then echo 'MultiModule: OK' ; else echo 'MULTIMODULE NOT DETECTED' ; fi
if ( rs-enumerate-devices | grep -q plugged ) ; then echo REALSENSE NOT DETECTED ; fi
if ( ! ps -aux  | grep -q "[b]aseboardlink.py" ) ; then echo BASEBOARDLINK NOT RUNNING ; fi
if ( ! ps -aux  | grep -q "[d]aemon.py" ) ; then echo DAEMON NOT RUNNING ; fi
if [ ! -f /home/pats/code/pats/base/build/executor ]; then echo 'EXECUTOR DOES NOT EXIST'; fi
if ( ! ps -aux  | grep -q "[e]xecutor" ) ; then echo EXECUTOR NOT RUNNING ; fi
if ! ( cd /home/pats/code/pats && git rev-parse HEAD | grep -q 9b61783b12fbd1677e2910e8ddfb4c6d8227239f ) ; then echo SHA DISCREPANCY; fi
