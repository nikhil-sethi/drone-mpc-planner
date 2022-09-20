#!/usr/bin/env bash

set -ex


if [[ $HOSTNAME != pats* ]]; then
	echo "Change hostname before running this script!"
	echo "!!! Note: this script is no longer ment to run on your personal laptop !!!"
	exit 1
fi

mkdir -p ~/dependencies
mkdir -p ~/code
mkdir -p ~/pats/sockets
mkdir -p ~/pats/xml
mkdir -p ~/pats/data
mkdir -p ~/pats/jsons
mkdir -p ~/pats/renders
mkdir -p ~/pats/logs
mkdir -p ~/pats/flags
mkdir -p ~/pats/status
mkdir -p ~/pats/images

KERNEL=$(uname -r)
ubuntu_str=$(lsb_release -a | grep Release)

if [ ! -f ~/dependencies/ssh_keys.done ] ; then

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

	[ -f ~/dependencies/pats_ssh_files_v4.tar.xz ] || {
		mkdir -p ~/.ssh
		cp pats_ssh_files_v4.tar.xz ~/.ssh
		pushd ~/.ssh

		tar -xf pats_ssh_files_v4.tar.xz
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
DEPENDENCIES_FLAG=dependencies-packages-v1.20.done
[ -f $DEPENDENCIES_FLAG ] || {
	sudo apt update
	sudo apt install -y build-essential g++ gdb libva-dev libswresample-dev libavutil-dev pkg-config libcurl4-openssl-dev ncdu openssh-server ffmpeg unattended-upgrades inotify-tools cpputest python3-pip dfu-util exfat-utils vnstat ifmetric net-tools lm-sensors nethogs htop git nano screen autossh usb-modeswitch moreutils cmake vainfo intel-gpu-tools lsb-core uptimed astyle
	
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		if [[ $KERNEL == "5.11."* ]] || [[ $KERNEL == "5.8."* ]]; then
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

	touch $DEPENDENCIES_FLAG
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

	if ([[ $KERNEL == "5.11."* ]] || [[ $KERNEL == "5.8."* ]]) && [ ! -f librealsense-kernel-patch_v1.2.done ]; then
		[ -d ./librealsense ] || {
			git clone git@github.com:IntelRealSense/librealsense.git
		}
		pushd librealsense/
		git checkout development
		./scripts/patch-realsense-ubuntu-lts.sh
		popd
		touch librealsense-kernel-patch_v1.2.done
	fi

	touch librealsense-packages_v1.1.done
}

[ -f python-packages-v1.3.done ] || {
	pip3 install cython pyserial types-pytz
	pip3 install numpy pandas scipy sklearn tqdm pause
	pip3 install xmltodict requests # for huawei-hilink-status
	pip3 install torch torchvision umap bioinfokit # deep learning

	#vscode linting and formatting:
	pip3 install pylint flake8 mypy autopep8
	if [[ $ubuntu_str != *"18.04"* ]] ; then
		pip3 install bandit
	fi

	pip3 install types-python-dateutil types-requests tzupdate

	touch python-packages-v1.3.done
}

[ -f dnn-dependencies-packages-v1.0.done ] || {
	pip3 install matplotlib
}

# Install command center packages
[ -f cc-dependencies-packages-v1.1.done ] || {
	sudo apt install -y python3-pyqt5 python3-pyqt5.qtmultimedia python3-pyqt5.qtquick
	sudo apt install -y ansible ansible-lint
	touch cc-dependencies-packages-v1.1.done
}

if [[ $ubuntu_str != *"18.04"* ]] && [[ ! -f gstreamer-v1.18.5.done ]]; then
	pushd ~/code/
	[ -d pats ] || {
		git clone git@github-pats:pats-drones/pats.git # needed for the patch
		pushd pats
		popd
	}
	popd
	git clone https://gitlab.freedesktop.org/gstreamer/gst-build.git
	pushd gst-build/
	git checkout 1.18.5
	meson builddir -Dvaapi=enabled -Dgst_debug=false -Dgstreamer-vaapi:with_x11=no --buildtype=release --prefix=/usr/local
	ninja -C builddir
	sudo ninja install -C builddir
	sudo ldconfig
	popd
	touch gstreamer-v1.18.5.done
fi

FFMPEG_FLAG=ffmpeg-v1.done
if [[ $ubuntu_str != *"18.04"* ]] && [[ ! -f $FFMPEG_FLAG ]]; then
	sudo apt-get update -qq && sudo apt install -y autoconf automake build-essential cmake git-core libass-dev libfreetype6-dev libgnutls28-dev libmp3lame-dev libsdl2-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev libxcb-shm0-dev libxcb-xfixes0-dev meson ninja-build pkg-config texinfo wget yasm zlib1g-dev libunistring-dev libaom-dev libunistring-dev libaom-dev libx264-dev nasm libx265-dev libnuma-dev libvpx-dev
	wget -O ffmpeg-snapshot.tar.bz2 https://ffmpeg.org/releases/ffmpeg-snapshot.tar.bz2
	tar -xf ffmpeg-snapshot.tar.bz2
	pushd ffmpeg
	./configure --extra-libs="-lpthread -lm" --ld="g++" --enable-gpl --enable-gnutls --enable-libaom --enable-libass --enable-libfreetype --enable-libvorbis --enable-libvpx --enable-libx264 --enable-libx265 --enable-nonfree 
	time make -j$(nproc)
	sudo make install
	sudo ldconfig
	popd
	touch $FFMPEG_FLAG
fi

# Uninstall openCV 3
[ ! -f opencv-3.4.2.done ] || {
	pushd opencv-3.4.2
	pushd build
	sudo make uninstall
	sudo ldconfig
	popd
	popd
	rm opencv-3.4.2* -rf
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

[ -f git.done ] || {
	git config --global push.default simple
	git config --global user.email "${HOSTNAME}@pats.com"
	git config --global user.name $HOSTNAME
	touch git.done
}

[ -f git_aliases_v1.0.done ] || {
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

QPOASES_FLAG=qpOASES-v1.done
[ -f $QPOASES_FLAG ] || {
	[ -d qpoases ] || {
		git clone git@github.com:coin-or/qpOASES.git
	}
	pushd qpOASES
	mkdir -p build
	pushd build
	cmake -DBUILD_SHARED_LIBS=1 ..
	make -j4
	sudo make install
	sudo ldconfig
	popd
	popd
	touch $QPOASES_FLAG
}
EIGEN_FLAG=Eigen-v1.done
[ -f $EIGEN_FLAG ] || {
	[ -d eigen ] || {
		git clone https://gitlab.com/libeigen/eigen.git
	}
	pushd eigen
	git checkout 3.4
	mkdir -p build
	pushd build
	cmake -DBUILD_SHARED_LIBS=1 ..
	make -j4
	sudo make install
	sudo ldconfig
	popd
	popd
	touch $EIGEN_FLAG
}

[ -f pats_code_v1.1.done ] || {
	echo Warning: manual config change required!
	exit
}

# Install the Pats code
PATS_CODE_FLAG=pats_code_v1.2.done
[ -f $PATS_CODE_FLAG ] || {
	touch ~/pats/flags/disable
	touch ~/pats/flags/disable_baseboard

	pushd ../code/
	[ -d ../code/pats ] || {
		git clone git@github-pats:pats-drones/pats.git
	}
	pushd pats
	mkdir -p base/build
	pushd base/build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j$(nproc)
	popd
	popd
	popd

	if [ ! ~/.ssh/config ] && [ ! ~/.ssh/config_tmp ]; then
		mv ~/.ssh/config_tmp ~/.ssh/config
	fi

	touch $PATS_CODE_FLAG
}

SYMLINK_FLAG=symlinks-v1.6.done
# Create nice symlinks
[ -f $SYMLINK_FLAG ] || {
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
		sudo cp /etc/ssh/sshd_config{,.bak} --backup=numbered
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
		sudo cp /etc/rc.local{,.bak} --backup=numbered
		sudo rm /etc/rc.local
	}
	sudo ln -s ~/code/pats/base/install/rc.local /etc/rc.local

	[ -f /etc/environment ] && {
		sudo cp /etc/environment{,.bak} --backup=numbered
		sudo rm /etc/environment
	}
	ubuntu_str=$(lsb_release -a | grep Release)
	if [[ $ubuntu_str == *"18.04"* ]] ; then
		sudo ln -s ~/code/pats/base/install/environment_18.04 /etc/environment
	else
		sudo ln -s ~/code/pats/base/install/environment_20.04 /etc/environment
	fi

	rm ~/.ssh/config -f
	ln -s ~/code/pats/base/install/sshconfig ~/.ssh/config

	[ -f ~/.gdbinit ] && {
		sudo cp ~/.gdbinit{,.bak} --backup=numbered
		sudo rm ~/.gdbinit
	}
	sudo ln -s ~/code/pats/base/install/.gdbinit ~/.gdbinit

	sudo cp /etc/sudoers{,.bak} --backup=numbered
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

	mkdir -p ~/Arduino
	[ -d ~/Arduino/libraries ] && {
		cp -r ~/Arduino/libraries{,.bak} --backup=numbered
		rm -r ~/Arduino/libraries
	}
	ln -s ~/code/pats/Arduino/libraries ~/Arduino/libraries

	touch $SYMLINK_FLAG
}

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
echo todo:
echo 1. Set bios to startup always at power on
echo 2. Add phone wifi ssid with: sudo nmcli device wifi connect !SSID! password !PASS!
echo 3. Change hostname stuff
echo "***********************************************************"



# Install pats-c dev packages
# # Install the Dash code
# PATSC_CODE_FLAG=patsc_code_v1.0.done
# [ -f $PATSC_CODE_FLAG ] || {
	

# 	pushd ./code/
# 	[ -d ../code/dash ] || {
# 		git clone git@github-dash:pats-drones/dash.git
# 	}
# 	popd

# 	touch $PATSC_CODE_FLAG
# }


# [ -f patsc-dependencies-packages-v1.1.done ] || {
# 	pip3 install -r ~/code/dash/patsc/requirements.txt
# 	touch patsc-dependencies-packages-v1.1.done
# }

# # Install dev packages
# [ -f dev-dependencies-packages-v1.1.done ] || {
# 	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
# 	sudo apt-get -y install apt-transport-https
# 	#to install sublime
# 	#echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
# 	#sudo snap install sublime-text --classic
# 	sudo snap install code --classic
# 	sudo apt update
# 	sudo apt install -y libqt5opengl5 libqt5opengl5-dev meld gitk git-gui terminator jstest-gtk
# 	#libgtk2.0-dev libtbb-dev qt5-default libgtkgl* libgtkgl2.0-* libgtkglext1 libgtkglext1-dev libgtkglext1-dev libgtkgl2.0-dev libgtk2.0-dev libgtk-3-dev gnome-devel
# 	touch dev-dependencies-packages-v1.1.done
# }

