#!/usr/bin/env bash

set -ex


# Install the Pats code
# You need to manually rsync a ssh key to the system at ~/.ssh/id_rsa.pats  first!
# e.g. rsync ~/.ssh/id_rsa.pats pats34:.ssh/ -a
# e.g. rsync ~/.ssh/id_rsa.trapeye pats34:.ssh/ -a


mkdir -p ~/code/
cd ~/code
git clone git@github-pats:pats-drones/pats.git
cd pats
mkdir -p base/build
cd base/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

cd ~/code 
git clone git@github-trapeye:pats-drones/trapeye.git
cd trapeye/basestation
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

rm ~/pats/release/build -rf
rm ~/pats/release/install -rf
rm ~/pats/release/scripts -rf
rm ~/pats/release/xml -rf
mkdir -p ~/pats/release/build
ln -s ~/code/pats/base/build/executor ~/pats/release/build/executor
ln -s ~/code/trapeye/basestation/build/trapeye ~/pats/release/build/trapeye
ln -s ~/code/pats/base/install ~/pats/release/install
ln -s ~/code/pats/base/scripts ~/pats/release/scripts
ln -s ~/code/pats/base/xml ~/pats/release/xml
touch ~/pats/flags/disable_updates

