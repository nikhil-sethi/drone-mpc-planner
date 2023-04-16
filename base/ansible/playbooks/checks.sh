#!/usr/bin/env bash


# cd ~/code/pats && echo git sha: $(git rev-parse HEAD)
# echo image version: $(cat ~/dependencies/image_version) kernel: $(uname -r) $(lsb_release -d)
# lscpu | grep 'Model name' &&
# tail ~/pats/logs/all_errors.log.20220810 -n +2
# rm ~/pats/data/dl_* -rf
# cd ~/pats/flags
# rm buf_overflow_realsense_flag
# rm daemon_wdt_flag
# rm fps_warning_flag
# rm proces_wdt_flag
# rm ~/pats/status/monitor_tmp.jpg
# rm ~/pats/logs/term.log
# rm ~/data_json_old -r
# rm ~/Downloads -rf
# rm ~/Desktop -rf
# rm ~/Music -rf
# rm ~/Pictures -rf
# rm ~/Videos -rf
# rm ~/code/pats/base/install/sshd_config.ucf-dist  -rf
# rm ~/code/pats/base/install/20auto-upgrades.ucf-dist -rf
# rm ~/code/pats/base/install/50unattended-upgrades.ucf-dist -rf
# rm ~/code/pats/base/install/sshd_config.ucf-dist -rf
# rm ~/code/pats/base/build_render -rf

#uptime -p
#lsb_release -a | grep Release
#lscpu | grep "Model name:"

if [ -f '/home/pats/pats/flags/disable' ] ; then echo 'SYSTEM DISABLED' ; fi
#if [ -f /var/run/reboot-required ]; then echo 'Reboot required'; fi
if [ -f '/home/pats/pats/xml/pats.xml' ] ; then
    if ( ! grep -q "Version=\"1.20\"" /home/pats/pats/xml/pats.xml ) ; then echo pats.xml VERSION ERROR ; fi
fi
if [ -e '/dev/baseboard' ] && [ -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD AVAILABLE BUT DISABLED' ; fi
if [ ! -e '/dev/baseboard' ] && [ ! -f '/home/pats/pats/flags/disable_baseboard' ] ; then echo 'BASEBOARD NOT DETETECTED AND NOT DISABLED' ; fi
#if [ -e '/dev/pats_mm0' ] ; then echo 'MultiModule: OK' ; else echo 'MULTIMODULE NOT DETECTED' ; fi
#if ( rs-enumerate-devices | grep -q plugged ) ; then echo REALSENSE NOT DETECTED ; fi
#if [ ! -f /home/pats/release/build/executor ]; then echo 'EXECUTOR DOES NOT EXIST'; fi
#if ( ! ps -aux  | grep -q "[e]xecutor" ) ; then echo EXECUTOR NOT RUNNING ; fi
if ( ! ps -aux  | grep -q "[b]aseboardlink.py" ) ; then echo BASEBOARDLINK NOT RUNNING ; fi
if ( ! ps -aux  | grep -q "[d]aemon.py" ) ; then echo DAEMON NOT RUNNING ; fi
if ( ! ifconfig | grep -q "inet 10.13" ) ; then echo WIREGUARD NOT CONNECTED; fi
#if ! ( cd /home/pats/code/pats && git rev-parse HEAD | grep -q 98ab746062614a94f1840b301f414aceca8c0c71 ) ; then echo SHA DISCREPANCY; fi
if [ ! -f '/home/pats/dependencies/release_upgraded_20230405.done' ] ; then echo 'RELEASE UPGRADE NOT FINISHED' ; fi