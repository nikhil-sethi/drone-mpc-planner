#!/usr/bin/env bash

set -e

cp -r ~/code/pats/base/install ~/code/release-18/
cp -r ~/code/pats/base/scripts ~/code/release-18/
cp -r ~/code/pats/base/xml ~/code/release-18/
cp -r ~/code/pats/config/firmwares/baseboard ~/code/release-18/
#cp -r ~/code/trapeye/basestation/scripts/setup_wifi ~/code/release-18/scripts/

cp -r ~/code/pats/base/install ~/code/release-20/
cp -r ~/code/pats/base/scripts ~/code/release-20/
cp -r ~/code/pats/base/xml ~/code/release-20/
cp -r ~/code/pats/config/firmwares/baseboard ~/code/release-20/
#cp -r ~/code/trapeye/basestation/scripts/setup_wifi ~/code/release-20/scripts/


#cd ~/code/release-18/build
#rsync pats241:code/pats/base/build/executor ./ -az
#cd ..
#rsync pats241:dependencies/binaries.tar.gz ./ -az

#cd ~/code/release-20/build
#rsync pats31:code/pats/base/build/executor ./ -az
#cd ..
#rsync pats31:dependencies/binaries.tar.gz ./ -az


