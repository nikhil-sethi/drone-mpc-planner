#!/usr/bin/env bash

#uptime -p
#lsb_release -a | grep Release
#lscpu | grep "Model name:"

if [ -f '/home/pats/pats/flags/disable' ] ; then echo 'SYSTEM DISABLED' ; fi
if [ -f /var/run/reboot-required ]; then echo 'Reboot required'; fi
if [ -f '/home/pats/pats/xml/pats.xml' ] ; then
    if ( ! grep -q "Version=\"1.21\"" /home/pats/pats/xml/pats.xml ) ; then echo pats.xml VERSION ERROR ; fi
fi
if [ -e '/dev/baseboard' ] && [ -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD AVAILABLE BUT DISABLED' ; fi
if [ ! -e '/dev/baseboard' ] && [ ! -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD NOT DETETECTED AND NOT DISABLED' ; fi
# if [ -e '/dev/pats_mm0' ] ; then echo 'MultiModule: OK' ; else echo 'MULTIMODULE NOT DETECTED' ; fi
# if [ ! -f '/home/pats/pats/flags/disable_executor' ] && ( rs-enumerate-devices | grep -q plugged ) ; then echo REALSENSE NOT DETECTED ; fi # may get stuck?!

if [ ! -f '/home/pats/pats/flags/disable_executor' ] && ( ! ps -aux  | grep -q "[e]xecutor" ) ; then echo EXECUTOR NOT RUNNING ; fi
if ( ! ps -aux  | grep -q "[b]aseboardlink.py" ) ; then echo BASEBOARDLINK NOT RUNNING ; fi
if ( ! ps -aux  | grep -q "[d]aemon.py" ) ; then echo DAEMON NOT RUNNING ; fi
if ( ! ifconfig | grep -q "inet 10.13" ) ; then echo WIREGUARD NOT CONNECTED; fi
if [ -f '/home/pats/dependencies/release_upgraded_20230405.done' ] ; then if ! ( cd /home/pats/pats/release && git describe --tags | grep -q "v2.3.3" ) ; then echo TAG DISCREPANCY; fi ; fi
if [ ! -f '/home/pats/dependencies/release_upgraded_20230405.done' ] && [ -f '/home/pats/dependencies/release_upgraded_P1_20230405.done' ] ; then echo 'RELEASE UPGRADE NOT FINISHED' ; fi
if [ ! -f '/home/pats/dependencies/release_upgraded_20230405.done' ] && [ ! -f '/home/pats/dependencies/release_upgraded_P1_20230405.done' ] ; then echo 'RELEASE UPGRADE NOT DONE' ; fi
