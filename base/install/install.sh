#!/usr/bin/env bash

set -ex

if [[ $HOSTNAME != pats* ]]; then
	echo "Change hostname before running this script!"
	echo "!!! Note: this script is no longer ment to run on your personal laptop !!!"
	exit 1
fi

PRE_COMPILED_BINARIES_PATH=/usr/local #in case of a non-standard build target, add it to /etc/ld.conf.d
KERNEL=$(uname -r)
ubuntu_str=$(lsb_release -a | grep Release)

mkdir -p ~/dependencies
sudo mkdir -p $PRE_COMPILED_BINARIES_PATH
sudo chown $USER $PRE_COMPILED_BINARIES_PATH
mkdir -p ~/pats/sockets
mkdir -p ~/pats/xml
mkdir -p ~/pats/data
mkdir -p ~/pats/jsons
mkdir -p ~/pats/renders
mkdir -p ~/pats/logs
mkdir -p ~/pats/flags
mkdir -p ~/pats/status
mkdir -p ~/pats/images

pushd ~/dependencies

SSH_KEYS_FLAG=ssh_keys.done
[ -f $SSH_KEYS_FLAG ] || {

	#Change hostname:
	#Update: sudo nano /etc/hosts
	#Change 127.0.1.1 pats*
	#Make it persistant, update: sudo nano /etc/hostname
	if [[ $HOSTNAME == pats* ]]; then
		echo "Hostname changed detected, good."
	else
		echo "Change hostname before running this script!"
		exit 1
	fi

	[ -f pats_ssh_files_v5.tar.xz ] || {
		mkdir -p ~/.ssh
		cp pats_ssh_files_v5.tar.xz ~/.ssh
		pushd ../.ssh

		tar -xf pats_ssh_files_v5.tar.xz
		eval `ssh-agent -s`

		# this is not necessary if the ssh keys from the tar package are correct already
		#chmod 600 ~/.ssh/pats_id_rsa
		#ssh-add ~/.ssh/pats_id_rsa

		[ -f config ] || { #if the pats repo does not exist yet, temporarily move the config to prevent git clone problems (which uses ssh)
			mv config config_tmp
		}

		popd
	}

	touch $SSH_KEYS_FLAG
}


DEPENDENCIES_FLAG=dependencies-packages-v1.22.done
[ -f $DEPENDENCIES_FLAG ] || {
	sudo apt update
	sudo apt install -y build-essential g++ gdb libva-dev libswresample-dev libavutil-dev pkg-config libcurl4-openssl-dev ncdu openssh-server unattended-upgrades inotify-tools cpputest python3-pip dfu-util exfat-utils vnstat ifmetric net-tools lm-sensors nethogs htop git nano screen autossh usb-modeswitch moreutils cmake vainfo intel-gpu-tools lsb-core uptimed astyle wireguard openresolv avrdude
	
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		if [[ $KERNEL == "5.11."* ]] || [[ $KERNEL == "5.8."* ]] || [[ $KERNEL == "5.15."* ]]; then
			sudo apt remove -y intel-media-va-driver
			sudo apt install -y intel-media-va-driver-non-free
		else
			sudo apt remove -y intel-media-va-driver intel-media-va-driver-non-free
		fi
		# gstreamer compile packages:
		sudo apt install -y libudev-dev nasm meson ninja-build flex bison libdrm-dev		
		sudo apt remove -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 libgstreamer-plugins-good1.0-0 gstreamer1.0-vaapi libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev		
	else
		sudo apt install -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 libgstreamer-plugins-good1.0-0 gstreamer1.0-vaapi libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
	fi

	#specific to enable opencv features and optimizations:
	sudo apt install -y yasm gfortran libjpeg8-dev libpng-dev libtiff-dev libatlas-base-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev libatlas3-base libatlas-base-dev liblapack3 liblapacke liblapacke-dev liblapack-dev ccache qtbase5-dev
	sudo apt-get remove -y modemmanager	
	sudo apt purge -y snapd # remove snap, because it uses data

	#for ffmpeg compilation
	sudo apt remove -y ffmpeg libavfilter-dev
	sudo apt install -y autoconf automake libass-dev libfreetype6-dev libgnutls28-dev libmp3lame-dev libsdl2-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev libxcb-shm0-dev libxcb-xfixes0-dev meson ninja-build pkg-config texinfo wget yasm zlib1g-dev libunistring-dev libx264-dev nasm libx265-dev libnuma-dev libvpx-dev 
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		sudo apt install -y libaom-dev
	fi

	touch $DEPENDENCIES_FLAG
}

TRAPEYE_FLAG=trapeye-v1.0.done
[ -f $TRAPEYE_FLAG ] || {
	sudo apt install -y hostapd isc-dhcp-server rfkill
	touch $TRAPEYE_FLAG
}

REALSENSE_FLAG=librealsense-packages_v1.1.done
[ -f $REALSENSE_FLAG ] || {

	sudo apt install -y software-properties-common
	sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
	sudo apt update

	# Install packages
	sudo apt install -y librealsense2-dkms librealsense2-dev librealsense2-dbg librealsense2-utils -y

	[ ! -f librealsense-packages.done ] || {
		#remove old ppa:
		sudo add-apt-repository --remove "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main"
		rm librealsense-packages.done
	}

	if ([[ $KERNEL == "5.11."* ]] || [[ $KERNEL == "5.8."* ]] || [[ $KERNEL == "5.15."* ]]) && [ ! -f librealsense-kernel-patch_v1.2.done ]; then
		[ -d ./librealsense ] || {
			git clone git@github.com:IntelRealSense/librealsense.git
		}
		pushd librealsense/
		git checkout development
		./scripts/patch-realsense-ubuntu-lts.sh
		popd
		touch librealsense-kernel-patch_v1.2.done
	fi

	touch $REALSENSE_FLAG
}

PYTHON_PACKAGES_FLAG=python-packages-v1.3.done
[ -f $PYTHON_PACKAGES_FLAG ] || {
	pip3 install cython pyserial types-pytz
	pip3 install numpy pandas scipy sklearn tqdm pause
	pip3 install xmltodict requests # for huawei-hilink-status
	pip3 install torch torchvision umap bioinfokit # deep learning
	pip3 install matplotlib

	#vscode linting and formatting:
	pip3 install pylint flake8 mypy autopep8
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		pip3 install bandit
	fi

	pip3 install types-python-dateutil types-requests tzupdate

	touch $PYTHON_PACKAGES_FLAG
}

PATS_RELEASE_FLAG=pats_release_v1.0.done
[ -f $PATS_RELEASE_FLAG ] || {
		touch ~/pats/flags/disable
		touch ~/pats/flags/disable_baseboard
		pushd ../pats
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		[ -d pats ] || {
			git clone git@github-release-20:pats-drones/release-20.git
			ln -s release-20 release
		}
	else
		[ -d pats ] || {
			git clone git@github-release-18:pats-drones/release-18.git
			ln -s release-18 release
		}
	fi
	popd
	if [ ! ~/.ssh/config ] && [ ! ~/.ssh/config_tmp ]; then
		mv ~/.ssh/config_tmp ~/.ssh/config
	fi
	touch $PATS_RELEASE_FLAG
}


GIT_FLAG=git.done
[ -f $GIT_FLAG ] || {
	git config --global push.default simple
	git config --global user.email "${HOSTNAME}@pats.com"
	git config --global user.name $HOSTNAME
	touch $GIT_FLAG
}

GIT_ALIASES_FLAG=git_aliases_v1.0.done
[ -f $GIT_ALIASES_FLAG ] || {
	git config --global alias.co checkout
	git config --global alias.br branch
	git config --global alias.ci commit
	git config --global alias.st status
	git config --global alias.dt "difftool -d"
	git config --global alias.ls "log --oneline"

	git config --global alias.s status
	git config --global alias.ss "status --short"
	git config --global alias.ssb "status --short --branch"
	git config --global alias.pr "pull --rebase"
	git config --global alias.cp "cherry-pick"

	touch $GIT_ALIASES_FLAG
}
VPN_FLAG=vpn-v1.done
# Create nice symlinks
[ -f $VPN_FLAG ] || {
	sudo mkdir -p /etc/wireguard/
	#wg genkey | tee /etc/wireguard/privatekey | wg /etc/wireguard/pubkey > /etc/wireguard/publickey
	sudo systemctl enable wg-quick@wg0.service
    touch $VPN_FLAG
}

SYMLINK_FLAG=symlinks-v2.2.done
# Create nice symlinks
[ -f $SYMLINK_FLAG ] || {
	[ -f ~/.screenrc ] && {
		cp ~/.screenrc{,.bak} --backup=numbered
		rm ~/.screenrc
	}
	ln -s ~/pats/release/install/.screenrc ~/

	[ -f ~/.bashrc ] && {
		cp ~/.bashrc{,.bak} --backup=numbered
		rm ~/.bashrc
	}
	ln -s ~/pats/release/install/.bashrc ~/

	[ -f /etc/ssh/sshd_config ] && {
		sudo cp /etc/ssh/sshd_config{,.bak} --backup=numbered
		sudo rm /etc/ssh/sshd_config
	}
	sudo ln -s ~/pats/release/install/sshd_config /etc/ssh/

	[ -f /etc/network/interfaces ] && {
			sudo cp /etc/network/interfaces{,.bak} --backup=numbered
			sudo rm /etc/network/interfaces
	}
	sudo ln -s ~/pats/release/install/interfaces /etc/network/interfaces

	[ -f /etc/apt/apt.conf.d/50unattended-upgrades ] && {
			sudo rm /etc/apt/apt.conf.d/50unattended-upgrades
	}
	sudo ln -s ~/pats/release/install/50unattended-upgrades /etc/apt/apt.conf.d/


	[ -f /etc/apt/apt.conf.d/10periodic ] && {
			sudo rm /etc/apt/apt.conf.d/10periodic
	}
	sudo ln -s ~/pats/release/install/10periodic /etc/apt/apt.conf.d/


	[ -f /etc/apt/apt.conf.d/20auto-upgrades ] && {
			sudo rm /etc/apt/apt.conf.d/20auto-upgrades
	}
	sudo ln -s ~/pats/release/install/20auto-upgrades /etc/apt/apt.conf.d/


	[ -f /etc/rc.local ] && {
		sudo cp /etc/rc.local{,.bak} --backup=numbered
		sudo rm /etc/rc.local
	}
	sudo ln -s ~/pats/release/install/rc.local /etc/rc.local

	[ -f /etc/environment ] && {
		sudo cp /etc/environment{,.bak} --backup=numbered
		sudo rm /etc/environment
	}
	ubuntu_str=$(lsb_release -a | grep Release)
	if [[ $ubuntu_str == *"18.04"* ]] ; then
		sudo ln -s ~/pats/release/install/environment_18.04 /etc/environment
	else
		sudo ln -s ~/pats/release/install/environment_20.04 /etc/environment
	fi

	rm ~/.ssh/config -f
	ln -s ~/pats/release/install/sshconfig ~/.ssh/config
	rm ~/.ssh/pats_ssh_config -f

	[ -f ~/.gdbinit ] && {
		sudo cp ~/.gdbinit{,.bak} --backup=numbered
		sudo rm ~/.gdbinit
	}
	sudo ln -s ~/pats/release/install/.gdbinit ~/.gdbinit

	sudo cp /etc/sudoers{,.bak} --backup=numbered
	sudo cp ~/pats/release/install/sudoers /etc/sudoers


	[ -f /etc/NetworkManager/NetworkManager.conf ] && {
		sudo cp /etc/NetworkManager/NetworkManager.conf{,.bak} --backup=numbered
		sudo rm /etc/NetworkManager/NetworkManager.conf
	}
	sudo ln -s ~/pats/release/install/NetworkManager.conf /etc/NetworkManager/NetworkManager.conf

	[ -f /etc/netplan/networkmanager.yaml ] && {
		sudo cp /etc/netplan/networkmanager.yaml{,.bak} --backup=numbered
		sudo rm /etc/netplan/networkmanager.yaml
	}
	sudo ln -s ~/pats/release/install/networkmanager.yaml /etc/netplan/networkmanager.yaml
	sudo netplan generate
	sudo netplan apply
	sudo service NetworkManager restart

	[ -f /etc/systemd/system.conf ] && {
		sudo cp /etc/systemd/system.conf{,.bak} --backup=numbered
		sudo rm /etc/systemd/system.conf
	}
	sudo ln -s ~/pats/release/install/system.conf /etc/systemd/system.conf

	sudo chown $USER $PRE_COMPILED_BINARIES_PATH -R
	sudo touch /etc/cloud/cloud-init.disabled

	sudo systemctl restart ssh.service

	touch $SYMLINK_FLAG
}

PATS_SYS_FLAG=pats_sys_config-v1.0.done
[ -f $PATS_SYS_FLAG ] || {
	# Add to groups
	sudo usermod -a -G dialout $USER
	sudo usermod -a -G video $USER
	echo "alias df='df -h -x squashfs -x tmpfs -x devtmpfs'" >> ~/.bash_aliases

	touch $PATS_SYS_FLAG
}

MM_FLAG=mm_install_v2.0.done
[ -f $MM_FLAG ] || {
	[ -f /lib/udev/rules.d/45-pats_mm.rules ] && {
		sudo rm /lib/udev/rules.d/45-pats_mm.rules
	}
	sudo ln -s ~/pats/release/install/45-pats_mm.rules /lib/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	touch $MM_FLAG
}

BB_FLAG=baseboard_install_v2.0.done
[ -f $BB_FLAG ] || {
	[ -f /lib/udev/rules.d/99-charging-pads.rules ] && {
		sudo rm /lib/udev/rules.d/99-charging-pads.rules
	}
	sudo ln -s ~/pats/release/install/99-charging-pads.rules /lib/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	touch $BB_FLAG
}


PATS_BIN_FLAG=pats_bin-v1.done
[ -f $PATS_BIN_FLAG ] || {
	tar -xf ~/pats/release/binaries.tar.xz
	rm /usr/local/* -rf
	mv ./usr/local/* /usr/local/
	sudo ldconfig
	sudo apt-get install -y $(cat ~/pats/release/package_list.txt | awk '{print $1"=" $2}')
	pip install -r ~/pats/release/requirements.txt
	touch $PATS_BIN_FLAG
}

popd



sudo apt-get autoremove -y
sudo apt-get clean -y

set +x
echo "********************** ALL DONE ***************************"
echo todo:
echo 1. Set bios to startup always at power on
echo 2. Add phone wifi ssid with: sudo nmcli device wifi connect !SSID! password !PASS!
echo 3. Change hostname stuff
echo "***********************************************************"

