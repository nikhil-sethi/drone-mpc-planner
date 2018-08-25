#!/usr/bin/env python2

#This daemon script automatically detects and fixes the 4g Huawei driver issue, that causes the stick to boot into a mass storage device instead of 4g modem
#Requires root!
#add line  to /etc/rc.local: python2 PATH/fix_4f_driver.py & 
#Made by Kevin

from __future__ import print_function
import time
import os
from subprocess import Popen
import sys
import signal
import subprocess


INTERVAL = 10

while True: # loops INTERVAL
	time_str =time.strftime("%I:%M:%S") + ' ' + time.strftime("%d/%m/%Y")
	proc = subprocess.Popen(['lsusb'],stdout=subprocess.PIPE) 	

	while True: # read lsusb output
		line = proc.stdout.readline()
		if line != '':

			if 'Huawei Technologies Co., Ltd. E353/E3131 (Mass storage mode)' in line:
				print(time_str + ': Huawei = HOER. Going to shout at it!')
				subprocess.Popen(['usb_modeswitch', '-v', '12d1', '-p', '1f01' ,'-M', '55534243123456780000000000000a11062000000000000100000000000000'])				
				#subprocess.Popen(['echo', 'hoerhoer'])			
		else: #end of lsusb output
			break

	
	time.sleep(INTERVAL)
