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

if [[ $ubuntu_str != *"18.04"* ]] ; then
	[ -d ~/pats/release-20 ] || {
		echo Clone repo 20 please.
		rm -rf ~/pats/release
		ln -s ~/pats/release-20 ~/pats/release
		cd ~/pats
		export -p
		git clone git@github-release-20:pats-drones/release-20.git
		cd ~
	}
else
	[ -d ~/pats/release-18 ] || {
		echo Clone repo 18 please.
		rm -rf ~/pats/release
		ln -s ~/pats/release-18 ~/pats/release
		cd ~/pats
		export -p
		git clone git@github-release-18:pats-drones/release-18.git
		cd ~
	}
fi


[ -f $RELEASE_UPGRADE_P4_FLAG ] || {
	cd ~/dependencies
	tar -xf ~/pats/release/binaries.tar.xz
	rm /usr/local/* -rf
	mv ./usr/local/* /usr/local/
	pip3 install --upgrade pip
	pip install --upgrade pip
	if [ ! ~/.ssh/config ] && [ ! ~/.ssh/config_tmp ]; then
		mv ~/.ssh/config_tmp ~/.ssh/config
	fi

	rm ~/.screenrc -rf
	ln -s ~/pats/release/install/.screenrc ~/
	rm ~/.bashrc -rf
	ln -s ~/pats/release/install/.bashrc ~/
	rm ~/.ssh/config -rf
	ln -s ~/pats/release/install/sshconfig ~/.ssh/config


	echo Release upgrade almost done.
	touch $RELEASE_UPGRADE_P4_FLAG
}


[ -f $RELEASE_UPGRADE_P5_FLAG ] || {

	rm ~/Desktop -rf
	rm ~/Downloads -rf
	rm ~/EPoBEg3XUAEC3rc.jpeg -rf
	rm ~/Music -rf
	rm ~/Pictures -rf
	rm ~/Public -rf
	rm ~/snap -rf
	rm ~/Templates -rf
	rm ~/Videos -rf
	rm ~/pats/flags/no_realsense_flag -rf
	rm ~/code -rf
	rm ~/trapeye/images/to_sent -rf
	rm ~/.ssh/pats_ssh_files_v*.tar.* -rf

	mkdir -p ~/pats/sockets
	mkdir -p ~/pats/xml
	mkdir -p ~/pats/data
	mkdir -p ~/pats/jsons
	mkdir -p ~/pats/renders
	mkdir -p ~/pats/logs
	mkdir -p ~/pats/flags
	mkdir -p ~/pats/status
	mkdir -p ~/pats/images
	mkdir -p ~/trapeye
	mkdir -p ~/trapeye/images
	mkdir -p ~/trapeye/db
	mkdir -p ~/trapeye/conf

	touch $RELEASE_UPGRADE_P5_FLAG
}
