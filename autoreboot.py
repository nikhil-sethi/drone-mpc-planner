#!/usr/bin/env python
import sys
import os
import subprocess
import datetime
import time

last_reboot_time = datetime.datetime.now()
while(1):
    now = datetime.datetime.now()    
    diff = (now - last_reboot_time).total_seconds()
    if now.hour == 14 and last_reboot_time.day != now.day:
    #   print('Reboot now: ' + str(diff))
        subprocess.call('reboot -r -t 60', shell=True)
    #else:
        #print('Current: ' + str(now.day) + ' reboot: ' + str(last_reboot_time.day))
    time.sleep(10)
