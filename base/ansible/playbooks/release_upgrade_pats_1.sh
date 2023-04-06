#!/usr/bin/env bash
set -e

ubuntu_str=$(lsb_release -a | grep Release)
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P1_FLAG=/home/pats/dependencies/release_upgraded_P1_20230405.done
RELEASE_UPGRADE_P2_FLAG=/home/pats/dependencies/release_upgraded_P2_20230405.done
RELEASE_UPGRADE_P3_FLAG=/home/pats/dependencies/release_upgraded_P3_20230405.done
RELEASE_UPGRADE_P4_FLAG=/home/pats/dependencies/release_upgraded_P4_20230405.done
RELEASE_UPGRADE_P5_FLAG=/home/pats/dependencies/release_upgraded_P5_20230405.done


if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi
if [ ! -f $RELEASE_UPGRADE_P2_FLAG ]; then
    echo RELEASE_UPGRADE_P2_FLAG not yet set!
    exit 0
fi

unset SSH_AUTH_SOCK
unset SSH_CLIENT
unset SSH_CONNECTION
unset SSH_TTY
export HOME=/home/pats/

[ -f $RELEASE_UPGRADE_P3_FLAG ] || {
	touch ~/pats/flags/disable
	touch ~/pats/flags/disable_trapeye
	cd ~/.ssh
	rsync dash:base_upgrade_tmp/known_hosts ./ -a
	rsync dash:.ssh/id_rsa.release-18 ./ -a
	rsync dash:.ssh/id_rsa.release-20 ./ -a
	rsync dash:base_upgrade_tmp/sshconfig ~/.ssh/sshconfig
	rm ~/.ssh/config
	mv ~/.ssh/sshconfig ~/.ssh/config
	eval "$(ssh-agent -s)"
	ssh-add id_rsa.release-18
	ssh-add id_rsa.release-20
	mv ~/dependencies ~/dependencies.old
	mkdir -p ~/dependencies
	mv ~/dependencies.old/*.done ~/dependencies/
	mv ~/dependencies.old/hostname_set ~/dependencies/ | true
	mv ~/dependencies.old/wireguard_set ~/dependencies/ | true
	mv ~/dependencies.old/timezone_set ~/dependencies/ | true
	mv ~/dependencies.old/image_version ~/dependencies/

	touch $RELEASE_UPGRADE_P3_FLAG
}
