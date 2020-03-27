#!/usr/bin/env python3
import time
import serial

ser = serial.Serial(port='/dev/ttyACM0')
time.sleep(0.1)
ser.write('#'.encode())
time.sleep(1)
ser.write('bl\n'.encode())

