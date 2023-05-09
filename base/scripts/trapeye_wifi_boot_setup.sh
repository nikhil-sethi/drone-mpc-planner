#!/bin/bash
# autostarted from rfkill-unblock.service
sleep 10

output=$(nmcli device status)
while read -r line; do
    if [[ $line == *wifi* ]]; then
        name=$(echo "$line" | awk '{print $1}')
        break
    fi
done <<< "$output"
wifi_adapter_name="$name"

/usr/sbin/rfkill unblock wlan
service hostapd stop
service isc-dhcp-server stop
sleep 1
ifconfig $wifi_adapter_name down
sleep 1
ifconfig $wifi_adapter_name 10.10.0.1/24 up
ifconfig $wifi_adapter_name 10.10.0.1/24 up
service hostapd start
service isc-dhcp-server start
