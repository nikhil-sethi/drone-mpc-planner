#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
	echo "Usage: $./configure_trapeye.sh [enable=0,1]"
	exit 1
fi

set -ex
if [[ $1 -eq 1 ]] ; then
	echo "Enabling trapeye config..."
    rm ~/pats/flags/disable_trapeye -rf
    sudo rm /etc/network/interfaces
    sudo ln -s ~/pats/release/install/interfaces_trapeye /etc/network/interfaces
else
    echo "Disabling trapeye config..."
    touch ~/pats/flags/disable_trapeye -rf
    sudo rm /etc/network/interfaces
    sudo ln -s ~/pats/release/install/interfaces /etc/network/interfaces
fi
sudo service NetworkManager restart


