#!/bin/bash
set -ex
working_dir=`pwd`

# Add librealsense repository
[ -f librealsense-packages.done ] || {
	sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
	sudo apt update

	# Install packages
	sudo apt install librealsense2-dkms librealsense2-dev librealsense2-dbg librealsense2-utils libva-dev libswresample-dev libavutil-dev pkg-config libjpeg9-dev htop git vim nano screen g++ cmake autossh usb-modeswitch -y
	touch librealsense-packages.done
	sudo apt-get autoremove
	sudo apt-get clean
}

# Create nice symlinks
mkdir -p /home/pats/code
[ -f /home/pats/.screenrc ] || {
	ln -s /home/pats/code/pats/config/.screenrc /home/pats/
}
# Add to groups
sudo usermod -a -G dialout pats

# Configure git
git config --global push.default simple
git config --global user.email "patsproto@pats.com"
git config --global user.name $HOSTNAME
sh ./git_alias.sh
# Copy id_rsa and id_rsa.pub to ~/.ssh

# Create dependencies directory
mkdir -p ~/dependencies
cd ~/dependencies

# Install packages
[ -f dependencies-packages.done ] || {
	sudo apt install cmake g++ libva-dev libswresample-dev libavutil-dev pkg-config libjpeg9-dev libcurl4-openssl-dev -y
	sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa   gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav -y libgstreamer-plugins-base1.0-* libgstreamer-plugins-bad1.0-* libgstreamer-plugins-good1.0-*
	sudo apt-get autoremove
	sudo apt-get clean
	touch dependencies-packages.done
}

# Install cmake with https support (required for opencv contrib)
[ -f cmake-3.11.4.done ] || {
	[ -d cmake-3.11.4 ] || {
		wget https://cmake.org/files/v3.11/cmake-3.11.4.tar.gz
		tar -xvf cmake-3.11.4.tar.gz				
	}
	pushd cmake-3.11.4
	./bootstrap --parallel=$(nproc) --system-curl
	make -j $(nproc)	
	popd
	touch cmake-3.11.4.done
}

# Install openCV
[ -f opencv-3.4.2.done ] || {
	[ -d opencv-3.4.2 ] || {
		wget https://github.com/opencv/opencv/archive/3.4.2.tar.gz
		mv 3.4.2.tar.gz opencv-3.4.2.tar.gz
		tar -xf opencv-3.4.2.tar.gz		
	}
	[ -d opencv_contrib-3.4.2 ] || {
		wget https://github.com/opencv/opencv_contrib/archive/3.4.2.tar.gz		
		mv 3.4.2.tar.gz opencv_contrib-3.4.2.tar.gz
		tar -xf opencv_contrib-3.4.2.tar.gz
	}
	pushd opencv-3.4.2
	mkdir -p build
	pushd build
	cmake -DCMAKE_BUILD_TYPE=release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.2/modules .. 
	time make -j$(nproc)
	sudo make install
	popd
	popd
	touch opencv-3.4.2.done
}

# Install the Outback vision code
cd ~/
[ -d code/pats ] || {
	git clone git@github.com:kevindehecker/pats.git
}
pushd code/pats
mkdir -p pc/build
cd ~/code/pats/pc/build
cmake ..
make -j4

# Autostart everything the folowing line:
# /home/up/obc_vision/config/daemon.sh
# to /etc/rc.local
