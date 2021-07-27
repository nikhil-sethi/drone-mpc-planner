#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
	echo "Usage: $./install.sh [install_monitoring_system=0,1]"
	exit 1
fi

if [[ $1 -eq 1 ]] ; then
	echo "Starting install install script for monitoring systems!"
fi

set -ex
mkdir -p ~/dependencies
mkdir -p ~/code

CPU_str=$(lscpu | grep -i 'model name' | uniq)
CPU=0
if [[ $CPU_str == *"AMD"* ]]; then
	CPU=1
elif [[ $CPU_str == *"i3-7100U"* ]]; then
	CPU=1
elif [[ $CPU_str == *"i3-8109U"* ]]; then
	CPU=1
elif [[ $CPU_str == *"i3-1115G4"* ]]; then
	CPU=2
fi
echo "CPU type: $CPU"

if [ ! -f ~/dependencies/ssh_keys.done ] && [ $1 -eq 1 ] ; then

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

	[ -f ~/dependencies/pats_ssh_files_v3.1.tar.xz ] || {
		mkdir -p ~/.ssh
		cp pats_ssh_files_v3.1.tar.xz ~/.ssh
		pushd ~/.ssh

		tar -xf pats_ssh_files_v3.1.tar.xz
		eval `ssh-agent -s`

		# this is not necessary if the ssh keys from the tar package are correct already
		#chmod 600 ~/.ssh/pats_id_rsa
		#ssh-add ~/.ssh/pats_id_rsa

		[ -f config ] || { #if the pats repo does not exist yet, temporarily move the config to prevent git clone problems (which uses ssh)
			mv config config_tmp
		}

		popd
	}

	touch ~/dependencies/ssh_keys.done
fi
pushd ~/dependencies

# Install pats dependency packages
[ -f dependencies-packages-v1.15.done ] || {
	sudo apt update
	sudo apt install -y build-essential g++ gdb libva-dev libswresample-dev libavutil-dev pkg-config libcurl4-openssl-dev ncdu openssh-server ffmpeg unattended-upgrades inotify-tools cpputest python3-pip dfu-util exfat-utils vnstat ifmetric net-tools lm-sensors nethogs htop git nano screen autossh usb-modeswitch moreutils cmake vainfo intel-gpu-tools
	if [ $CPU -eq 1 ]; then
		# gstreamer packages:
		sudo apt install -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 libgstreamer-plugins-good1.0-0 gstreamer1.0-vaapi libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
	elif [ $CPU -eq 2 ]; then
		sudo apt install -y intel-media-va-driver-non-free
		# gstreamer compile packages:
		sudo apt install -y libudev-dev nasm meson ninja-build flex bison libdrm-dev
	fi

	#specific to enable opencv features and optimizations:
	sudo apt install -y yasm gfortran libjpeg8-dev libpng-dev libtiff-dev libatlas-base-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev libatlas3-base libatlas-base-dev liblapack3 liblapacke liblapacke-dev liblapack-dev ccache qtbase5-dev

	sudo apt-get remove -y modemmanager

	if [[ $1 -eq 1 ]] ; then
		sudo apt purge -y snapd # remove snap, because it uses data
	fi

	touch dependencies-packages-v1.15.done
}

# Add librealsense repository
[ -f librealsense-packages_v1.1.done ] || {

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

	if [ $CPU -eq 2 ] && [ ! -f librealsense-kernel-patch_v1.0.done ]; then
		git clone git@github.com:IntelRealSense/librealsense.git
		pushd librealsense/
		#./scripts/setup_udev_rules.sh
		./scripts/patch-realsense-ubuntu-lts.sh
		popd
		touch librealsense-kernel-patch_v1.0.done
	fi

	touch librealsense-packages_v1.1.done
}

[ -f python-packages-v1.0.done ] || {
	pip3 install cython pyserial
	pip3 install numpy pandas scipy sklearn tqdm pause
	pip3 install xmltodict requests # for huawei-hilink-status
	pip3 install torch torchvision umap bioinfokit # deep learning
	touch python-packages-v1.0.done
}

# Install dev packages
if [[ $1 -eq 0 ]] ; then
	[ -f dev-dependencies-packages-v1.1.done ] || {
		wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
		sudo apt-get install apt-transport-https
		#to install sublime
		#echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
		#sudo snap install sublime-text --classic
		sudo snap install code --classic
		sudo apt update
		sudo apt install -y libqt5opengl5 libqt5opengl5-dev astyle  meld gitk git-gui terminator jstest-gtk
		#libgtk2.0-dev libtbb-dev qt5-default libgtkgl* libgtkgl2.0-* libgtkglext1  libgtkglext1-dev libgtkglext1-dev libgtkgl2.0-dev  libgtk2.0-dev libgtk-3-dev gnome-devel
		touch dev-dependencies-packages-v1.1.done
	}

	[ -f dnn-dependencies-packages-v1.0.done ] || {
		pip3 install matplotlib
	}

	# Install command center packages
	[ -f cc-dependencies-packages-v1.0.done ] || {
		sudo apt install -y python3-pyqt5 python3-pyqt5.qtmultimedia python3-pyqt5.qtquick
		touch cc-dependencies-packages-v1.0.done
	}
fi

if [ $CPU -eq 2 ] && [ $1 -eq 1 ] && [ ! -f gstreamer-v1.18.4.done ]; then
	pushd ~/code/
	[ -d pats ] || {
		git clone git@github.com:pats-drones/pats.git # needed for the patch
		pushd pats
		popd
	}
	popd
	git clone https://gitlab.freedesktop.org/gstreamer/gst-build.git
	pushd gst-build/
	git checkout 1.18.4
	meson builddir -Dvaapi=enabled -Dgst_debug=false -Dgstreamer-vaapi:with_x11=no --buildtype=release --prefix=/usr/local
	patch gst-plugins-base/gst-libs/gst/gl/meson.build ~/code/pats/base/install/gst_1.18.4_fix.patch
	ninja -C builddir
	sudo ninja install -C builddir
	sudo ldconfig
	popd
	touch gstreamer-v1.18.4.done
fi

# Uninstall openCV 3
[ ! -f opencv-3.4.2.done ] || {
	pushd opencv-3.4.2
	pushd build
	sudo make uninstall
	sudo ldconfig
	popd
	popd
	rm  opencv-3.4.2* -rf
}

# Install openCV
if [ ! -f opencv-4.5.2.done ] && [ ! -f opencv-4.3.0.done ] ; then
	[ -d opencv-4.5.2 ] || {
		wget https://github.com/opencv/opencv/archive/4.5.2.tar.gz
		mv 4.5.2.tar.gz opencv-4.5.2.tar.gz
		tar -xf opencv-4.5.2.tar.gz
	}
	pushd opencv-4.5.2
	mkdir -p build
	pushd build
	cmake -DCMAKE_BUILD_TYPE=release -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_FFMPEG=OFF ..
	[ -h ~/dependencies/gst-build/ ] || { #gstreamer 1.18.4 seems to cause some ffmpeg compile problem with opencv
		cmake -DWITH_FFMPEG=OFF ..
	}
	time make -j$(nproc)
	sudo make install
	sudo ldconfig
	popd
	popd
	touch opencv-4.5.2.done
fi

if [[ $1 -eq 1 ]] ; then
	[ -f git.done ] || {
		# Configure git
		git config --global push.default simple
		git config --global user.email "${HOSTNAME}@pats.com"
		git config --global user.name $HOSTNAME
		touch git.done
	}
fi

[ -f git_aliases_v1.0.done ] || {

	#sudo apt install bash-completion
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

	touch git_aliases_v1.0.done
}

# Install the Pats code
[ -f pats_code_v1.1.done ] || {
	pushd ../code/
	[ -d ../code/pats ] || {
		git clone git@github.com:pats-drones/pats.git
	}
	pushd pats
	mkdir -p base/build
	pushd base/build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j$(nproc)
	popd
	popd
	popd

	if [ ! ~/.ssh/config ] && [ $1 -eq 1 ] && [ ! ~/.ssh/config_tmp ]; then
		mv ~/.ssh/config_tmp ~/.ssh/config
	fi

	touch pats_code_v1.1.done
}

if [[ $1 -eq 1 ]] ; then

	mkdir -p ~/pats/data
	mkdir -p ~/pats/jsons
	mkdir -p ~/pats/renders
	mkdir -p ~/pats/logs
	mkdir -p ~/pats/flags
	mkdir -p ~/pats/status
	mkdir -p ~/pats/images
	touch ~/pats/flags/disable

	# Create nice symlinks
	[ -f symlinks-v1.3.done ] || {
		[ -f ~/.screenrc ] && {
			cp ~/.screenrc{,.bak} --backup=numbered
			rm ~/.screenrc
		}
		ln -s ~/code/pats/base/install/.screenrc ~/

		[ -f ~/.bashrc ] && {
			cp ~/.bashrc{,.bak} --backup=numbered
			rm ~/.bashrc
		}
		ln -s ~/code/pats/base/install/.bashrc ~/

		[ -f /etc/ssh/sshd_config ] && {
			sudo cp  /etc/ssh/sshd_config{,.bak} --backup=numbered
			sudo rm /etc/ssh/sshd_config
		}
		sudo ln -s ~/code/pats/base/install/sshd_config /etc/ssh/

		[ -f /etc/network/interfaces ] && {
				sudo cp /etc/network/interfaces{,.bak} --backup=numbered
				sudo rm /etc/network/interfaces
		}
		sudo ln -s ~/code/pats/base/install/interfaces /etc/network/interfaces

		[ -f /etc/apt/apt.conf.d/50unattended-upgrades ] && {
				sudo rm /etc/apt/apt.conf.d/50unattended-upgrades
		}
		sudo ln -s ~/code/pats/base/install/50unattended-upgrades /etc/apt/apt.conf.d/


		[ -f /etc/apt/apt.conf.d/10periodic ] && {
				sudo rm /etc/apt/apt.conf.d/10periodic
		}
		sudo ln -s ~/code/pats/base/install/10periodic /etc/apt/apt.conf.d/


		[ -f /etc/apt/apt.conf.d/20auto-upgrades ] && {
				sudo rm /etc/apt/apt.conf.d/20auto-upgrades
		}
		sudo ln -s ~/code/pats/base/install/20auto-upgrades /etc/apt/apt.conf.d/


		[ -f /etc/rc.local ] && {
			sudo cp  /etc/rc.local{,.bak} --backup=numbered
			sudo rm /etc/rc.local
		}
		sudo ln -s ~/code/pats/base/install/rc.local /etc/rc.local

		rm ~/.ssh/config -f
		ln -s ~/code/pats/base/install/sshconfig ~/.ssh/config

		sudo cp  /etc/sudoers{,.bak} --backup=numbered
		sudo cp ~/code/pats/base/install/sudoers /etc/sudoers


		[ -f /etc/NetworkManager/NetworkManager.conf ] && {
			sudo cp /etc/NetworkManager/NetworkManager.conf{,.bak} --backup=numbered
			sudo rm /etc/NetworkManager/NetworkManager.conf
		}
		sudo ln -s ~/code/pats/base/install/NetworkManager.conf /etc/NetworkManager/NetworkManager.conf

		[ -f /etc/netplan/networkmanager.yaml ] && {
			sudo cp /etc/netplan/networkmanager.yaml{,.bak} --backup=numbered
			sudo rm /etc/netplan/networkmanager.yaml
		}
		sudo ln -s ~/code/pats/base/install/networkmanager.yaml /etc/netplan/networkmanager.yaml
		sudo netplan generate
		sudo netplan apply
		sudo service NetworkManager restart

		[ -f /etc/systemd/system.conf ] && {
			sudo cp /etc/systemd/system.conf{,.bak} --backup=numbered
			sudo rm /etc/systemd/system.conf
		}
		sudo ln -s ~/code/pats/base/install/system.conf /etc/systemd/system.conf
		

		sudo systemctl restart ssh.service


		touch symlinks-v1.3.done
	}
fi

[ -f pats_sys_config-v1.0.done ] || {
	# Add to groups
	sudo usermod -a -G dialout $USER
	sudo usermod -a -G video $USER
	echo "alias df='df -h -x squashfs -x tmpfs -x devtmpfs'" >> ~/.bash_aliases

	touch pats_sys_config-v1.0.done
}

#install driver for multi module:
[ -f mm_install_v1.1.done ] || {
	[ -f /lib/udev/rules.d/45-pats_mm.rules ] && {
		sudo rm /lib/udev/rules.d/45-pats_mm.rules
	}
	sudo ln -s ~/code/pats/base/install/45-pats_mm.rules /lib/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	touch mm_install_v1.1.done
}

#install driver for baseboard:
[ -f baseboard_install_v1.1.done ] || {
	[ -f /lib/udev/rules.d/99-charging-pads.rules ] && {
		sudo rm /lib/udev/rules.d/99-charging-pads.rules
	}
	sudo ln -s ~/code/pats/base/install/99-charging-pads.rules /lib/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	touch baseboard_install_v1.1.done
}

popd
sudo apt-get autoremove -y
sudo apt-get clean -y

set +x
echo "********************** ALL DONE ***************************"
if [[ $1 -eq 1 ]] ; then
	echo todo:
	echo 1. Set bios to startup always at power on
	echo 2. Add phone wifi ssid with: sudo nmcli device wifi connect !SSID! password !PASS!
	echo 3. Change hostname stuff
	echo "***********************************************************"
fi
