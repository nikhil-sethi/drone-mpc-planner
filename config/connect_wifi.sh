set -e
if [ $# -eq 0 ]
  then
    echo "Usage: connect_wifi.sh <SSID> <PASS>"
    exit 1
fi

sudo nmcli device wifi connect $1 password $2 &
sleep 60
killall pats
sleep 2
sudo rtcwake -m no -s 10
sudo swapoff -a
sudo systemctl poweroff
