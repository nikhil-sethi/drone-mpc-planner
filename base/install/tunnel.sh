#!/usr/bin/env bash
# First argument is the address
# Second argument the ssh port of the server

#direct log in: ssh -i ~/.ssh/id_rsa kevin@131.180.117.41 -t "ssh pats@localhost -p 16669 -i~/.ssh/id_rsa"
#tunnel for sftp: ssh -i ~/.ssh/id_rsa kevin@131.180.117.41 -L 6002:localhost:16669 -N

set -ex
# Settings
USER=pats
BASE_PORT=6000
IP=$1
HOSTNAME=$( hostname )
HOST_ID=$( hostname | tr -dc '0-9' )
PORT=$(( $BASE_PORT + $HOST_ID * 100 ))
SSH_PORT=${2:-22}

# Retry tunnel
echo "Creating tunnel at $IP:$SSH_PORT with port $PORT"
while [ 1 ]; do
 	/usr/bin/autossh -M 0 -o "ExitOnForwardFailure yes" -o "ServerAliveInterval 180" -o "ServerAliveCountMax 3" -C4NR $PORT:localhost:22 $USER@$IP -p $SSH_PORT

	echo waiting 10s for starting tunnel
	sleep 10s

done
