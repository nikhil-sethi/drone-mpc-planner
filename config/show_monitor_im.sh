#!/bin/bash

while true; do 
	rsync -avz -e ssh pats@192.168.0.101:/home/pats/code/pats/pc/build/monitor.png ~/monitor.png
	sleep 10
	echo hoi
done


#then open the file with:
#eog ~/monitor.png
#it will auto refresh