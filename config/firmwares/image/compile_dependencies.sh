#!/usr/bin/env bash

set -ex

PRE_COMPILED_BINARIES_PATH=/usr/local/ #in case of a non-standard build target, add it to /etc/ld.conf.d

#remove existing:
rm ~/dependencies -rf
sudo rm /usr/local -rf

sudo mkdir -p $PRE_COMPILED_BINARIES_PATH
sudo chown $USER $PRE_COMPILED_BINARIES_PATH
ubuntu_str=$(lsb_release -a | grep Release)
mkdir -p  ~/dependencies
pushd ~/dependencies
sudo ldconfig

#upgrade all and everything
sudo apt update 
sudo apt upgrade -y
pip freeze > requirements.txt
pip install --upgrade -r requirements.txt


if [[ $ubuntu_str != *"18.04"* ]] ; then
	GSTREAMER_FLAG=gstreamer-v1.18.6.done
	if [[ $ubuntu_str != *"18.04"* ]] && [[ ! -f $GSTREAMER_FLAG ]]; then
		git clone https://gitlab.freedesktop.org/gstreamer/gst-build.git
		pushd gst-build/
		git checkout 1.18.6
		meson build -Dvaapi=enabled -Dgst_debug=false -Dgstreamer-vaapi:with_x11=no --buildtype=release --prefix=$PRE_COMPILED_BINARIES_PATH
		ninja -C build
		ninja install -C build
		popd
		touch $GSTREAMER_FLAG
	fi
fi

if [[ $ubuntu_str != *"18.04"* ]] ; then
	FFMPEG_FLAG=ffmpeg-v1.done
	if [[ $ubuntu_str != *"18.04"* ]] && [[ ! -f $FFMPEG_FLAG ]]; then
		wget -O ffmpeg-snapshot.tar.bz2 https://ffmpeg.org/releases/ffmpeg-snapshot.tar.bz2
		tar -xf ffmpeg-snapshot.tar.bz2
		pushd ffmpeg
		./configure --prefix=$PRE_COMPILED_BINARIES_PATH --extra-libs="-lpthread -lm" --ld="g++" --enable-gpl --enable-gnutls --enable-libaom --enable-libass --enable-libfreetype --enable-libvorbis --enable-libvpx --enable-libx264 --enable-libx265 --enable-nonfree 
		time make -j$(nproc)
		make install
		popd
		touch $FFMPEG_FLAG
	fi
fi

OPENCV_FLAG=opencv-4.5.2.done
[ -f $OPENCV_FLAG ] || {
	[ -d opencv-4.5.2 ] || {
		wget https://github.com/opencv/opencv/archive/4.5.2.tar.gz
		mv 4.5.2.tar.gz opencv-4.5.2.tar.gz
		tar -xf opencv-4.5.2.tar.gz
	}
	pushd opencv-4.5.2
	mkdir -p build
	pushd build
	cmake -DCMAKE_BUILD_TYPE=release -DCMAKE_INSTALL_PREFIX=$PRE_COMPILED_BINARIES_PATH -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_FFMPEG=OFF ..
	time make -j$(nproc)
	make install
	popd
	popd
	touch $OPENCV_FLAG
}

QPOASES_FLAG=qpOASES-v1.done
[ -f $QPOASES_FLAG ] || {
	[ -d qpoases ] || {
		git clone git@github.com:coin-or/qpOASES.git
	}
	pushd qpOASES
	mkdir -p build
	pushd build
	cmake -DBUILD_SHARED_LIBS=1 -DCMAKE_INSTALL_PREFIX=$PRE_COMPILED_BINARIES_PATH ..
	time make -j$(nproc)
	make install
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
	cmake -DBUILD_SHARED_LIBS=1 -DCMAKE_INSTALL_PREFIX=$PRE_COMPILED_BINARIES_PATH ..
	time make -j$(nproc)
	make install
	popd
	popd
	touch $EIGEN_FLAG
}

sudo ldconfig

tar -cJvf binaries.tar.xz /usr/local
pip freeze > requirements.txt
# dpkg-query -f '${db:Status-Status} ${binary:Package} ${Version}\n' -W | awk '/^installed/ {print $2, $3}' > package_list.txt
dpkg -l | grep ^ii | awk '{print $2 "=" $3}' > package_list.txt
mkdir debs -p
pushd debs
cat ../package_list.txt | xargs -I {} apt-get download {}
popd
popd

cd
# Build the Executor
PATS_CODE_FLAG=pats_code_v1.3.done
[ -f $PATS_CODE_FLAG ] || {
	mkdir -p code
	pushd code
	[ -d pats ] || {
		git clone git@github-pats:pats-drones/pats.git
	}
	pushd pats
	mkdir -p base/build
	
	popd
	popd
	touch $PATS_CODE_FLAG
}
pushd code/pats/base/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

set +x
echo "********************** ALL DONE ***************************"
