#! /usr/bin/env bash

# Configuration
tunnels=( dash mavlab-gpu dinstech )
tunnels_port=( 22 22 16666 )

# Start tunnels
for i in ${!tunnels[@]}
do
	/usr/bin/screen -t 't'$i $i /bin/bash -c '~/code/pats/config/tunnel.sh '${tunnels[i]}' '${tunnels_port[i]}'; exec /bin/bash'
done

# Start other screens
i=$(($i + 1))
/usr/bin/screen -t co2 $i /bin/bash -c 'cd ~/code/Config/scripts && ~/code/Config/scripts/co2.py; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t down $i /bin/bash -c 'cd ~/code/pats/dashboard/ && ./downloader.py -rl; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t code $i /bin/bash -c 'cd ~/code/pats/; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t data $i /bin/bash -c 'cd ~/Downloads/moth_jsons; exec /bin/bash'
