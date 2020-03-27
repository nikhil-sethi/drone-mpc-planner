#!/usr/bin/env python3
import time
import serial

ser = serial.Serial(port='/dev/ttyACM0')
# time.sleep(10.1)
# print('hoer')
# ser.write('save\n'.encode())
# time.sleep(3.1)
# print('hoer')
# exit()

ser.write('#'.encode())
time.sleep(1)

with open('BF_Trashcan.txt', 'r') as fp:
	for cnt, line in enumerate(fp):
		print("Line {}: {}".format(cnt, line))
		ser.write(line.encode())
		ser.write('\n'.encode())
		time.sleep(0.05)
	ser.write('save\n'.encode())
	time.sleep(0.5)
	ser.close()
ser = serial.Serial(port='/dev/ttyACM0')
time.sleep(1)

print("Uploading settings successfull")
		



