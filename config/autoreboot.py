#!/usr/bin/env python
import sys
import os
import subprocess
import datetime
import time

last_reboot_time = datetime.datetime.now()
print('starting autorebooter: ' + str(last_reboot_time))
while(1):
    now = datetime.datetime.now()    
    diff = (now - last_reboot_time).total_seconds()
    if now.hour == 14 and last_reboot_time.day != now.day:
        print('Reboot now: ' + str(diff))
        subprocess.call('rtcwake -m off -s 5', shell=True)
        exit(0)
    #else:
        #print('Current: ' + str(now.day) + ' reboot: ' + str(last_reboot_time.day))
    time.sleep(10)