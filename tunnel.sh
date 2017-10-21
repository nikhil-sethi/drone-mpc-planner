#!/usr/bin/env bash

while [ 1 ]; do
	sleep 10s
	ssh -i /home/slamdunk/.ssh/id_rsa -fNR 16667:localhost:22 houjebek@dinstech.nl -p 16666
	#configure firewall to forward port 16666 to the correct nat ip
	#from that ip, do ssh slamdunk@localhost -p 16667
	#Add id_rsa.pub to the authorized_keys e.g. with ssh-copy-id
done
