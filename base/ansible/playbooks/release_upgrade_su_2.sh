#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive
export DEBIAN_PRIORITY=critical

ubuntu_str=$(lsb_release -a | grep Release)
APT_UPGRADE_FLAG=/home/pats/dependencies/apt_upgraded_20230405.done
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P1_FLAG=/home/pats/dependencies/release_upgraded_P1_20230405.done
RELEASE_UPGRADE_P2_FLAG=/home/pats/dependencies/release_upgraded_P2_20230405.done

if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi

if [[ $ubuntu_str == *"18.04"* ]] ; then
	echo Nothing to do for 18
else
    if [ ! -f $RELEASE_UPGRADE_P2_FLAG ]; then
        apt remove -y linux-headers-5.4.* linux-headers-5.4.*-generic linux-headers-5.11.0-43-generic linux-headers-5.15.0-50-generic linux-headers-5.15.0-58-generic  linux-hwe-5.11-headers-5.11.0-43 linux-hwe-5.15-headers-5.15.0-50 linux-hwe-5.15-headers-5.15.0-58 linux-image-5.15.0-50-generic linux-image-5.15.0-58-generic linux-modules-5.15.0-50-generic linux-modules-5.15.0-58-generic linux-modules-extra-5.15.0-50-generic linux-modules-extra-5.15.0-58-generic
        apt autoremove -y
        apt install librealsense2-dkms -y
    fi
fi
touch $RELEASE_UPGRADE_P2_FLAG