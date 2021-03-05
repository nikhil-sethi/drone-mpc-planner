#! /usr/bin/env bash

# Configuration
tunnels=( dash dinstech  )
tunnels_port=( 22 16666)

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 't'$i /bin/bash -c '~/code/pats/base/install/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

mkdir -p ~/pats/logs

# Start other screens
/usr/bin/screen -t upl /bin/bash -c 'cd ~/code/pats/base/data_processing && ~/code/pats/base/data_processing/daemon.py; exec /bin/bash'
/usr/bin/screen -t pats /bin/bash -c 'cd ~/code/pats/base/data_processing && ~/code/pats/base/install/autostart.sh 2>&1 | /usr/bin/tee -a ~/pats/logs/term.log; exec /bin/bash'
/usr/bin/screen -t bash /bin/bash -c 'cd ~/; exec /bin/bash'
