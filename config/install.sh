#!/bin/bash
set -ex
working_dir=`pwd`

# Create dependencies directory
mkdir -p ~/dependencies
mkdir -p ~/code
pushd ~/dependencies

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

# Install packages
[ -f dependencies-packages.done ] || {
	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
	sudo apt-get update
	
	sudo apt install -y cmake g++ libva-dev libswresample-dev libavutil-dev pkg-config libjpeg9-dev libcurl4-openssl-dev sublime-text -y 
	sudo apt install  -y openssh-server gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav libgstreamer-plugins-base1.0-* libgstreamer-plugins-bad1.0-* libgstreamer-plugins-good1.0-* terminator
	sudo apt-get remove -y modemmanager
	
	

	
	touch dependencies-packages.done
}

# Add librealsense repository
[ -f librealsense-packages.done ] || {
	sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
	sudo apt update

	# Install packages
	sudo apt install -y librealsense2-dkms librealsense2-dev librealsense2-dbg librealsense2-utils libva-dev libswresample-dev libavutil-dev pkg-config libjpeg9-dev htop git vim nano screen g++ cmake autossh usb-modeswitch -y
	touch librealsense-packages.done
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
	sudo make install	
	popd
	touch cmake-3.11.4.done
	exec bash
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

# Install the Pats code
[ -f pats_code.done ] || {
	[ -d ../code/pats ] || {
		git clone git@github.com:pats-drones/pats.git
	}
	pushd ../code/pats
	mkdir -p pc/build
	pushd pc/build
	cmake ..
	make -j4
	popd
	popd
	touch pats_code.done
}

# Add to groups
sudo usermod -a -G dialout $USER

# Configure git
git config --global push.default simple
git config --global user.email "patsproto@pats.com"
git config --global user.name $HOSTNAME
sh ~/code/pats/config/git_alias.sh
# Copy id_rsa and id_rsa.pub to ~/.ssh

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

	[ -f /etc/rc.local ] && {
		sudo cp  -n /etc/rc.local{,.bak}
		sudo rm /etc/rc.local
	}
	sudo ln -s ~/code/pats/config/rc.local /etc/rc.local

	sudo ln -s ~/code/pats/config/45-pats_mm.rules /lib/udev/rules.d/

	touch symlinks.done
}

popd

sudo apt-get autoremove -y
sudo apt-get clean -y

set +x
echo "***********************************************************"
echo All done!. Reboot, or run:
echo sudo systemctl restart ssh.service
echo sudo udevadm control --reload-rules && udevadm trigger
echo "***********************************************************"
