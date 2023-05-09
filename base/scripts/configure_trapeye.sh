#!/usr/bin/env bash
if [ "$#" -ne 1 ]; then
	echo "Usage: $./configure_trapeye.sh [enable=0,1]"
	exit 1
fi

set -ex
if [[ $1 -eq 1 ]] ; then
	echo "Enabling trapeye..."
    output=$(nmcli device status)
    while read -r line; do
        if [[ $line == *wifi* ]]; then
            name=$(echo "$line" | awk '{print $1}')
            break
        fi
    done <<< "$output"
    wifi_adapter_name="$name"

    rm ~/pats/flags/disable_trapeye -rf
    mkdir -p ~/trapeye/conf
    cd ~/trapeye/conf
    rm ~/trapeye/conf/* -rf
    cp ~/pats/release/install/trapeye/* ./
    find . -type f -exec sed -i "s/wlan0/$wifi_adapter_name/g" {} \;

    sudo nmcli connection delete $(nmcli connection show | grep wifi | grep -E -o '[0-9a-f\-]{36}') | true

    sudo cp dhcpd.conf /etc/dhcp/dhcpd.conf
    sudo cp isc-dhcp-server /etc/default/isc-dhcp-server
    sudo cp hostapd.conf /etc/hostapd/hostapd.conf
    sudo cp rfkill-unblock.service /etc/systemd/system/rfkill-unblock.service
    sudo rm /etc/network/interfaces
    sudo cp interfaces /etc/network/interfaces

    sudo systemctl unmask rfkill-unblock
    sudo systemctl unmask isc-dhcp-server
    sudo systemctl unmask hostapd

    sudo systemctl enable rfkill-unblock
    sudo systemctl enable isc-dhcp-server
    sudo systemctl enable hostapd

    sudo systemctl start rfkill-unblock
    sudo service isc-dhcp-server start
    sudo service hostapd start
else
    echo "Disabling trapeye..."
    touch ~/pats/flags/disable_trapeye
    sudo rm /etc/network/interfaces
    sudo ln -s ~/pats/release/install/interfaces /etc/network/interfaces
    sudo rm /etc/dhcp/dhcpd.conf
    sudo rm /etc/default/isc-dhcp-server
    sudo rm /etc/hostapd/hostapd.conf
    sudo rm /etc/systemd/system/rfkill-unblock.service
    sudo systemctl unmask rfkill-unblock
    sudo systemctl disable rfkill-unblock
    sudo systemctl stop rfkill-unblock
fi
echo All done. Now reboot.