#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive
export DEBIAN_PRIORITY=critical

ubuntu_str=$(lsb_release -a | grep Release)
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P1_FLAG=/home/pats/dependencies/release_upgraded_P1_20230405.done


IMAGE_VERSION=$(cat /home/pats/dependencies/image_version)
if [[ $IMAGE_VERSION == "NUC10+11 v3" ]]; then
    touch /home/pats/dependencies/release_upgraded_P1_20230405.done
fi

if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi

if [ ! -f $RELEASE_UPGRADE_P1_FLAG ]; then
	if [[ $ubuntu_str == *"18.04"* ]] ; then
		apt remove -y sendmail
		unattended-upgrade -d
        apt upgrade -y
        rm /home/pats/dependencies/grub_20
        mkdir -p /home/pats/dependencies
		mv /usr/local /home/pats/dependencies/ | true
		mkdir -p /usr/local/
		chown pats /usr/local/ -R
	else # ubu 20
        cp /home/pats/dependencies/grub_20 /etc/default/grub
        rm  /home/pats/dependencies/grub_20
		apt-mark hold grub-efi-amd64
		apt remove -y sendmail
        update-grub
		unattended-upgrade -d
        apt remove -y librealsense2-dkms
        apt upgrade -y
        #apt-mark unhold grub-efi-amd64

        mkdir -p /home/pats/dependencies
		mv /usr/local /home/pats/dependencies/ | true
		mkdir -p /usr/local/
		chown pats /usr/local/ -R
    fi
	apt install -y avrdude gdisk
	echo Please reboot me.
	touch $RELEASE_UPGRADE_P1_FLAG
fi
