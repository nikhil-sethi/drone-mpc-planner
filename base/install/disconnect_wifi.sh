set -e
if [ $# -eq 0 ]
  then
    echo "Usage: connect_wifi.sh <connection name>"
    echo "You can see the connection name by running nmcli. It's probably <SSID> <optionally some number>"
    exit 1
fi

killall ssh
sudo nmcli c delete id "$1" &
sleep 60
set +e
killall executor
killall ssh
sleep 2
sudo rtcwake -m no -s 20
sudo swapoff -a
sudo systemctl poweroff
