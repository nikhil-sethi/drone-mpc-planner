#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
	echo "Usage: $./install.sh [install_monitoring_system=0,1]"
	exit 1
fi

if [[ $1 -eq 1 ]] ; then
	echo "Starting install install script for monitoring systems!"
fi

set -ex
working_dir=`pwd`
# Create dependencies directory
mkdir -p ~/dependencies
mkdir -p ~/code

[ -f ssh_keys.done ] || {
	if [[ $1 -eq 1 ]] ; then
		[ -f ~/dependencies/pats_ssh_files.tar.xz ] || {
			cp pats_ssh_files.tar.xz ~/dependencies
		}

		#Change hostname: sudo hostname pats-proto1 
		#Update: sudo nano /etc/hosts 
		#Change 127.0.1.1 pats-proto1 
		#Make it persistant, update: sudo nano /etc/hostname
		if [[ $HOSTNAME == pats-proto* ]];
		then
		  echo "Hostname changed detected, good."
		else
		  echo "Change hostname before running this script!"
		  exit 1
		fi
	fi

	[ ! -f pats_ssh_files.tar.xz ] || {
			#add deploy key
		tar -xf pats_ssh_files.tar.xz
		mkdir -p ~/.ssh
		mv authorized_keys ~/.ssh/
		mv id_rsa ~/.ssh/pats_id_rsa
		chmod 600 ~/.ssh/pats_id_rsa
		mv id_rsa.pub ~/.ssh/pats_pats_id_rsa.pub
		mv known_hosts ~/.ssh/
		eval `ssh-agent -s`
		ssh-add ~/.ssh/pats_id_rsa
	}
}
pushd ~/dependencies

# Install pats dependency packages
[ -f dependencies-packages-v1.4.done ] || {
	sudo apt-get update  
	sudo apt install -y cmake g++ libva-dev libswresample-dev libavutil-dev pkg-config libcurl4-openssl-dev ncdu openssh-server ffmpeg unattended-upgrades inotify-tools cpputest python3-pip python-pip dfu-util exfat-utils
	sudo apt install -y gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-* libgstreamer-plugins-bad1.0-* libgstreamer-plugins-good1.0-* gstreamer1.0-vaapi vainfo
	sudo apt-get remove -y modemmanager
	pip3 install pyserial
	#specific to enable opencv features and optimizations:
	sudo apt install -y yasm gfortran libjpeg8-dev libpng-dev libtiff-dev libatlas-base-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev libatlas3-base libatlas-base-dev liblapack3 liblapacke liblapacke-dev liblapack-dev ccache
	pip3 install numpy pandas scipy sklearn tqdm
	touch dependencies-packages-v1.3.done
}

# Add librealsense repository
[ -f librealsense-packages.done ] || {
	sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
	sudo apt update

	# Install packages
	sudo apt install -y librealsense2-dkms librealsense2-dev librealsense2-dbg librealsense2-utils libva-dev libswresample-dev libavutil-dev pkg-config htop git vim nano screen g++ cmake autossh usb-modeswitch -y
	touch librealsense-packages.done
}

# Install dev packages
[ -f dev-dependencies-packages-v1.1.done ] || {
	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
	sudo apt-get install apt-transport-https
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
	sudo snap install sublime-text --classic
	sudo snap install code --classic
	sudo apt-get update
	sudo apt install -y libqt5opengl5 libqt5opengl5-dev astyle  meld gitk git-gui terminator jstest-gtk
	#libgtk2.0-dev libtbb-dev qt5-default libgtkgl* libgtkgl2.0-* libgtkglext1  libgtkglext1-dev libgtkglext1-dev libgtkgl2.0-dev  libgtk2.0-dev libgtk-3-dev gnome-devel
	touch dev-dependencies-packages-v1.1.done
}

# Install command center packages
[ -f cc-dependencies-packages-v1.0.done ] || {
	sudo apt install -y python3-pyqt5 python3-pyqt5.qtmultimedia python3-pyqt5.qtquick
	touch cc-dependencies-packages-v1.0.done
}

# Install cmake with https support (required for opencv contrib)
[ -f cmake-3.11.4.done ] || {
	[ -d cmake-3.11.4 ] || {
		wget https://cmake.org/files/v3.11/cmake-3.11.4.tar.gz
		tar -xvf cmake-3.11.4.tar.gz				
	}
	pushd cmake-3.11.4
	./bootstrap --parallel=$(nproc) --system-curl
	make -j$(nproc)
	sudo make install	
	popd
	touch cmake-3.11.4.done
	popd
	echo "Restarting Bash. I'm NOT finished. Start me again!"
	exec bash
}

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
[ -f opencv-4.3.0.done ] || {
	[ -d opencv-4.3.0 ] || {
		wget https://github.com/opencv/opencv/archive/4.3.0.tar.gz
		mv 4.3.0.tar.gz opencv-4.3.0.tar.gz
		tar -xf opencv-4.3.0.tar.gz		
	}
	pushd opencv-4.3.0
	mkdir -p build
	pushd build
	cmake -DCMAKE_BUILD_TYPE=release -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_QT=ON -DWITH_OPENGL=ON .. 
	time make -j$(nproc)
	sudo make install
	popd
	popd
	touch opencv-4.3.0.done
}

if [[ $1 -eq 1 ]] ; then
	[ -f git.done ] || {
		# Configure git
		git config --global push.default simple
		git config --global user.email "${HOSTNAME}@pats.com"
		git config --global user.name $HOSTNAME
	}
fi

# Install the Pats code
[ -f pats_code.done ] || {
	pushd ../code/
	[ -d ../code/pats ] || {
		git clone git@github.com:pats-drones/pats.git
	}
	pushd pats
	sh config/git_alias.sh
	mkdir -p pc/build
	pushd pc/build
	cmake ..
	make -j$(nproc)
	popd
	popd
	popd
	
	touch pats_code.done
}

if [[ $1 -eq 1 ]] ; then

	mkdir -p ~/data_json
	mkdir -p ~/data
	
	# Create nice symlinks
	[ -f symlinks.done ] || {
		[ -f ~/.screenrc ] && {
			cp -n ~/.screenrc{,.bak}
			rm ~/.screenrc
		}
		ln -s ~/code/pats/config/.screenrc ~/
		
		[ -f ~/.bashrc ] && {
			cp -n ~/.bashrc{,.bak}
			rm ~/.bashrc
		}
		ln -s ~/code/pats/config/.bashrc ~/

		[ -f /etc/ssh/sshd_config ] && {
			sudo cp  -n /etc/ssh/sshd_config{,.bak}
			sudo rm /etc/ssh/sshd_config
		}
		sudo ln -s ~/code/pats/config/sshd_config /etc/ssh/

		[ -f /etc/apt/apt.conf.d/50unattended-upgrades ] && {
				sudo cp  -n /etc/apt/apt.conf.d/50unattended-upgrades{,.bak}
				sudo rm /etc/apt/apt.conf.d/50unattended-upgrades
		}
		sudo ln -s ~/code/pats/config/50unattended-upgrades /etc/apt/apt.conf.d/

		[ -f /etc/rc.local ] && {
			sudo cp  -n /etc/rc.local{,.bak}
			sudo rm /etc/rc.local
		}
		sudo ln -s ~/code/pats/config/rc.local /etc/rc.local

		rm ~/.ssh/config -f
		ln -s ~/code/pats/config/sshconfig ~/.ssh/config
				
		sudo systemctl restart ssh.service
		
		touch symlinks.done
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
[ -f mm_install.done ] || {
	sudo ln -s ~/code/pats/config/45-pats_mm.rules /lib/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	touch mm_install.done
}

# [ -f analysis-dependencies-packages-v1.0.done ] || {
# 	curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
# 	sudo apt install -y libssl1.0-dev curl nodejs nodejs sqlitebrowser
# 	pushd ../code/pats/analysis/moth_watcher/
# 	npm install
# 	npm audit fix
# 	popd
# 	touch analysis-dependencies-packages-v1.0.done
# }

popd



sudo apt-get autoremove -y
sudo apt-get clean -y

set +x
echo "***********************************************************"
echo todo: 
echo 1. Set bios to startup always at power on
echo 2. Add phone wifi ssid with: sudo nmcli device wifi connect !SSID! password !PASS!
echo 3. Change hostname stuff
echo "***********************************************************"
