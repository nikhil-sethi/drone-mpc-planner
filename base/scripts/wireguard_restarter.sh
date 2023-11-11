#! /usr/bin/env bash
INTERFACE="wg0"

while true; do
    sleep 300  # Sleep for 5 minutes before checking again
    systemctl is-active --quiet wg-quick@$INTERFACE.service
    if [ $? -ne 0 ]; then
        echo "WireGuard interface is not active. Restarting..."
        systemctl restart wg-quick@$INTERFACE.service
    fi
done