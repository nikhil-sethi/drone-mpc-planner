#! /usr/bin/env bash

# Configuration
tunnels=( mavlab-gpu dinstech )
tunnels_port=( 22 16666 )

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 'tunnel'$i $i /bin/bash -c '~/code/pats/config/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

# Start other screens
i=$(($i + 1))
/usr/bin/screen -t pats $i /bin/bash -c '~/code/pats/config/autostart.sh 2>&1 | /usr/bin/tee -a ~/pats_daemon.log; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t config $i /bin/bash -c 'cd ~/code/pats/config; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t build $i /bin/bash -c 'cd ~/code/pats/pc/build; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t data $i /bin/bash -c 'cd ~/data; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t home $i /bin/bash -c 'cd ~/; exec /bin/bash'
