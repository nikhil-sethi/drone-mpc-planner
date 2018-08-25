#!/usr/bin/env bash
echo waiting 11s for starting tunnel

while [ 1 ]; do
	sleep 12s
	echo also starting backup tunnel!
	/usr/bin/autossh -M 21000 -i /home/pats/.ssh/id_rsa -NR 16668:localhost:22 kevin@131.180.117.41
 	#ssh -i /home/slamdunk/.ssh/id_rsa -NR 16667:localhost:22 houjebek@dinstech.nl -p 16666
	#configure firewall to forward port 16666 to the correct nat ip
	#from that ip, do ssh slamdunk@localhost -p 16667
	#Add id_rsa.pub to the authorized_keys e.g. with ssh-copy-id:
	#ssh-copy-id -i ~/.ssh/id_rsa.pub kevin@131.180.117.41
done
