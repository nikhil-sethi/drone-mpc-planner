#! /usr/bin/env bash

# Configuration
tunnels=( dash mavlab-gpu dinstech  )
tunnels_port=( 22 22 16666)

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 'tunnel'$i $i /bin/bash -c '~/code/pats/config/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

# Start other screens
i=$(($i + 1))
/usr/bin/screen -t staupd $i /bin/bash -c 'cd ~/code/pats/config && ~/code/pats/config/status_sender.py; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t pats $i /bin/bash -c 'cd ~/code/pats/config && ~/code/pats/config/autostart.sh 2>&1 | /usr/bin/tee -a ~/pats_daemon.log; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t src $i /bin/bash -c 'cd ~/code/pats/src; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t data $i /bin/bash -c 'cd ~/data; exec /bin/bash'
