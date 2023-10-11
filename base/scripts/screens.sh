#! /usr/bin/env bash

# Configuration
tunnels=( dash )
tunnels_port=( 22 )

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 't'$i /bin/bash -c '~/pats/release/scripts/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

mkdir -p ~/pats/logs
# Start other screens
# /usr/bin/screen -t hogs /bin/bash -c 'sleep 30 && sudo nethogs -v3; exec /bin/bash' && sleep 1
/usr/bin/screen -t dmn /bin/bash -c 'cd ~/pats/release/scripts && ~/pats/release/scripts/auto_daemon_starter.py && sleep 1; exec /bin/bash'
/usr/bin/screen -t bb /bin/bash -c 'cd ~/pats/release/scripts && ~/pats/release/scripts/auto_baseboardlink_starter.py  && sleep 1; exec /bin/bash'
/usr/bin/screen -t exe /bin/bash -c 'cd ~/pats/release/scripts && ~/pats/release/scripts/auto_executor_starter.sh 2>&1 | /usr/bin/tee -a ~/pats/logs/term.log && sleep 1; exec /bin/bash'
[ -f /home/pats/pats/flags/disable_trapeye ] || {
	/usr/bin/screen -t te /bin/bash -c '~/pats/release/scripts/auto_trapeye_starter.sh 30'
}
/usr/bin/screen -t prmtl /bin/bash -c 'cd ~/pats/release/logging && ~/pats/release/logging/promtail-linux-amd64 --config.file promtail-config.yaml && sleep 1; exec /bin/bash'
/usr/bin/screen -t loki /bin/bash -c 'cd ~/pats/release/logging && ~/pats/release/logging/loki-linux-amd64 --config.file loki-config.yaml && sleep 1; exec /bin/bash'
/usr/bin/screen -t bash /bin/bash -c 'cd ~/; exec /bin/bash'