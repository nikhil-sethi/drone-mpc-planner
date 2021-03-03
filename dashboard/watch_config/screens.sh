#! /usr/bin/env bash

# Configuration
tunnels=( dash dinstech )
tunnels_port=( 22 16666 )

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 't'$i /bin/bash -c '~/code/pats/base/install/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

# Start other screens
/usr/bin/screen -t co2 $i /bin/bash -c 'cd ~/code/Config/scripts && ~/code/Config/scripts/co2.py; exec /bin/bash'
/usr/bin/screen -t down $i /bin/bash -c 'cd ~/code/pats/dashboard/ && ./watch_config/downloader.py; exec /bin/bash'
/usr/bin/screen -t backup $i /bin/bash -c 'cd ~/code/pats/dashboard/ && ./watch_config/dash_backup.py; exec /bin/bash'
/usr/bin/screen -t bash $i /bin/bash -c 'cd ~/; exec /bin/bash'
