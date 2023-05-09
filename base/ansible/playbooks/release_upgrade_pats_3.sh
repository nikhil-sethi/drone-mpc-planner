#!/usr/bin/env bash
set -e

ubuntu_str=$(lsb_release -a | grep Release)
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P8_FLAG=/home/pats/dependencies/release_upgraded_P8_20230405.done

if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi

[ -f $RELEASE_UPGRADE_P8_FLAG ] || {
	if [ -e '/dev/baseboard' ] ; then
		touch ~/pats/flags/disable_baseboard
		kill $(pgrep -f baseboardlink.py) || true
		cd ~/pats/release/firmware/baseboard/
		./flash.sh
		rm ~/pats/flags/disable_baseboard
 	fi
 	rm ~/pats/flags/disable -rf
	kill $(pgrep -f daemon.py) || true
	killall executor || true
	touch $RELEASE_UPGRADE_P8_FLAG
}
touch $RELEASE_UPGRADE_FLAG
echo Release upgrade SUCCESSFUL