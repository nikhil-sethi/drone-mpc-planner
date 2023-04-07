#!/usr/bin/env bash
set -ex
export DEBIAN_FRONTEND=noninteractive
export DEBIAN_PRIORITY=critical

ubuntu_str=$(lsb_release -a | grep Release)
APT_UPGRADE_FLAG=/home/pats/dependencies/apt_upgraded_20230405.done
RELEASE_UPGRADE_FLAG=/home/pats/dependencies/release_upgraded_20230405.done
RELEASE_UPGRADE_P1_FLAG=/home/pats/dependencies/release_upgraded_P1_20230405.done
RELEASE_UPGRADE_P2_FLAG=/home/pats/dependencies/release_upgraded_P2_20230405.done
RELEASE_UPGRADE_P3_FLAG=/home/pats/dependencies/release_upgraded_P3_20230405.done
RELEASE_UPGRADE_P4_FLAG=/home/pats/dependencies/release_upgraded_P4_20230405.done
RELEASE_UPGRADE_P5_FLAG=/home/pats/dependencies/release_upgraded_P5_20230405.done
RELEASE_UPGRADE_P6_FLAG=/home/pats/dependencies/release_upgraded_P6_20230405.done
RELEASE_UPGRADE_P7_FLAG=/home/pats/dependencies/release_upgraded_P7_20230405.done

if [ -f $RELEASE_UPGRADE_FLAG ]; then
    echo Nothing to see here.
    exit 0
fi
if [ ! -f $RELEASE_UPGRADE_P1_FLAG ]; then
    echo RELEASE_UPGRADE_P1_FLAG not yet set!
    exit 0
fi
if [ ! -f $RELEASE_UPGRADE_P5_FLAG ]; then
    echo RELEASE_UPGRADE_P5_FLAG not yet set!
    exit 0
fi

cd /home/pats/dependencies
[ -f $RELEASE_UPGRADE_P6_FLAG ] || {
	apt install -y build-essential g++ gdb libva-dev libswresample-dev libavutil-dev pkg-config libcurl4-openssl-dev ncdu openssh-server unattended-upgrades inotify-tools cpputest python3-pip dfu-util exfat-utils vnstat ifmetric net-tools lm-sensors nethogs htop git nano screen autossh usb-modeswitch moreutils cmake vainfo intel-gpu-tools lsb-core uptimed astyle wireguard openresolv avrdude gdisk

	if [[ $ubuntu_str != *"18.04"* ]] ; then
		if [[ $KERNEL == "5.11."* ]] || [[ $KERNEL == "5.8."* ]] || [[ $KERNEL == "5.15."* ]]; then
			apt remove -y intel-media-va-driver
			apt install -y intel-media-va-driver-non-free
		else
			apt remove -y intel-media-va-driver intel-media-va-driver-non-free
		fi
		# gstreamer compile packages:
		apt install -y libudev-dev nasm meson ninja-build flex bison libdrm-dev
		apt remove -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 libgstreamer-plugins-good1.0-0 gstreamer1.0-vaapi libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
	else
		apt install -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 libgstreamer-plugins-good1.0-0 gstreamer1.0-vaapi libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
	fi

	#specific to enable opencv features and optimizations:
	apt install -y yasm gfortran libjpeg8-dev libpng-dev libtiff-dev libatlas-base-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev libatlas3-base libatlas-base-dev liblapack3 liblapacke liblapacke-dev liblapack-dev ccache qtbase5-dev

	apt-get remove -y modemmanager
	apt purge -y snapd # remove snap, because it uses data

	#for ffmpeg compilation
	apt remove -y ffmpeg libavfilter-dev
	apt install -y autoconf automake libass-dev libfreetype6-dev libgnutls28-dev libmp3lame-dev libsdl2-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev libxcb-shm0-dev libxcb-xfixes0-dev meson ninja-build pkg-config texinfo wget yasm zlib1g-dev libunistring-dev libx264-dev nasm libx265-dev libnuma-dev libvpx-dev
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		apt install -y libaom-dev
	fi

	touch $RELEASE_UPGRADE_P6_FLAG
}

TRAPEYE_FLAG=trapeye-v1.1.done
[ -f $TRAPEYE_FLAG ] || {
	apt install -y hostapd isc-dhcp-server rfkill
	touch $TRAPEYE_FLAG
}



# Create nice symlinks
[ -f $RELEASE_UPGRADE_P7_FLAG ] || {
	[ -f /etc/ssh/sshd_config ] && {
		cp /etc/ssh/sshd_config{,.bak} --backup=numbered
	}
	rm /etc/ssh/sshd_config -rf
	ln -s /home/pats/pats/release/install/sshd_config /etc/ssh/

	[ -f /etc/network/interfaces ] && {
			cp /etc/network/interfaces{,.bak} --backup=numbered
	}
	rm /etc/network/interfaces -rf
	ln -s /home/pats/pats/release/install/interfaces /etc/network/interfaces

	rm /etc/apt/apt.conf.d/50unattended-upgrades -rf
	ln -s /home/pats/pats/release/install/50unattended-upgrades /etc/apt/apt.conf.d/

	rm -rf /etc/apt/apt.conf.d/10periodic
	ln -s /home/pats/pats/release/install/10periodic /etc/apt/apt.conf.d/

	rm -rf /etc/apt/apt.conf.d/20auto-upgrades
	ln -s /home/pats/pats/release/install/20auto-upgrades /etc/apt/apt.conf.d/

	[ -f /etc/rc.local ] && {
		cp /etc/rc.local{,.bak} --backup=numbered
	}
	rm -rf /etc/rc.local
	ln -s /home/pats/pats/release/install/rc.local /etc/rc.local

	[ -f /etc/environment ] && {
		cp /etc/environment{,.bak} --backup=numbered
	}
	rm -rf /etc/environment
	if [[ $ubuntu_str == *"18.04"* ]] ; then
		ln -s /home/pats/pats/release/install/environment_18.04 /etc/environment
	else
		ln -s /home/pats/pats/release/install/environment_20.04 /etc/environment
	fi

	[ -f /home/pats/.gdbinit ] && {
		cp /home/pats/.gdbinit{,.bak} --backup=numbered
	}
	rm -rf /home/pats/.gdbinit
	ln -s /home/pats/pats/release/install/.gdbinit /home/pats/.gdbinit

	cp /etc/sudoers{,.bak} --backup=numbered
	cp /home/pats/pats/release/install/sudoers /etc/sudoers

	[ -f /etc/NetworkManager/NetworkManager.conf ] && {
		cp /etc/NetworkManager/NetworkManager.conf{,.bak} --backup=numbered
	}
	rm -rf /etc/NetworkManager/NetworkManager.conf
	ln -s /home/pats/pats/release/install/NetworkManager.conf /etc/NetworkManager/NetworkManager.conf

	[ -f /etc/netplan/networkmanager.yaml ] && {
		cp /etc/netplan/networkmanager.yaml{,.bak} --backup=numbered
	}
	rm -rf /etc/netplan/networkmanager.yaml
	ln -s /home/pats/pats/release/install/networkmanager.yaml /etc/netplan/networkmanager.yaml
	netplan generate
	netplan apply
	service NetworkManager restart

	[ -f /etc/systemd/system.conf ] && {
		cp /etc/systemd/system.conf{,.bak} --backup=numbered
	}
	rm -rf /etc/systemd/system.conf
	ln -s /home/pats/pats/release/install/system.conf /etc/systemd/system.conf

	if [[ $ubuntu_str != *"18.04"* ]] ; then
		touch /etc/cloud/cloud-init.disabled
	fi

	systemctl restart ssh.service

	rm /lib/udev/rules.d/45-pats_mm.rules -rf
	ln -s /home/pats/pats/release/install/45-pats_mm.rules /lib/udev/rules.d/
	rm /lib/udev/rules.d/99-charging-pads.rules -rf
	ln -s /home/pats/pats/release/install/99-charging-pads.rules /lib/udev/rules.d/
	udevadm control --reload-rules && udevadm trigger

	rm /home/pats/dependencies/grub_20 -rf
	rm /home/pats/5.11.0-38-generic-* -rf
	rm /home/pats/dependencies/usr -rf


	mkdir -p /home/pats/pats/updates
	chmod 755 /home/pats/pats/updates

	touch $RELEASE_UPGRADE_P7_FLAG
	echo Please reboot me
}

ldconfig
