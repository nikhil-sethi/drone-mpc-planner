#!/usr/bin/env bash

uptime -p
cd ~/code/pats && echo git sha: $(git rev-parse HEAD)
echo image version: $(cat ~/dependencies/image_version) kernel: $(uname -r) $(lsb_release -d)
lscpu | grep 'Model name' &&
if [ -f /var/run/reboot-required ]; then echo 'reboot required'; fi
if [ -f '/home/pats/pats/xml/pats.xml' ] ; then echo 'Custom pats.xml found' ; fi
if ( ! grep -q "Version=\"1.20\"" /home/pats/pats/xml/pats.xml ) ; then echo pats.xml VERSION ERROR ; fi
if [ -e '/dev/baseboard' ] && [ -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'Baseboard AVAILABLE BUT DISABLED' ; fi
if [ ! -e '/dev/baseboard' ] && [ ! -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'Baseboard NOT DETETECTED AND NOT DISABLED' ; fi
if [ -e '/dev/pats_mm0' ] ; then echo 'MultiModule: OK' ; else echo 'MultiModule: NOT DETECTED' ; fi
if ( rs-enumerate-devices | grep -q plugged ) ; then echo NO REALSENSE ; fi
if [ ! -f /home/pats/code/pats/base/build/executor ]; then echo 'EXECUTOR DOES NOT EXIST'; fi
if ( ! ps -aux  | grep -q "[e]xecutor" ) ; then echo NO EXECUTOR ; fi
if ( cd /home/pats/code/pats && git rev-parse HEAD | grep -q 5925856ef1e87bac1e77700d9bc52fa0457644c2 ) ; then echo sha ok ; else echo SHA DISCREPANCY; fi
