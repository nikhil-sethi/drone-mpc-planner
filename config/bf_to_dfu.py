#!/usr/bin/env python3
import time
import serial

ser = serial.Serial(port='/dev/ttyACM0')
time.sleep(0.1)
hoer = '#'
ser.write(hoer.encode())
time.sleep(1)
hoer = 'bl\n'
ser.write(hoer.encode())

