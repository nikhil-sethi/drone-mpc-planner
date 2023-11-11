#!/usr/bin/env python3
import sys
import os
import re
import subprocess
import stat
import argparse
import time
import serial  # pip3 install pyserial or if you use conda: conda install -c anaconda pyserial


def dev_not_exists(path):
    try:
        return stat.S_ISBLK(os.stat(path).st_mode)
    except:
        return True


def spinning_cursor():
    while True:
        for cursor in '|/-\\':
            yield cursor


def reset_to_dfu(port):
    ser = serial.Serial(port=port)
    time.sleep(0.1)
    cmd = 'gcc ./upload-reset.c'
    subprocess.call(cmd, shell=True)
    cmd = './a.out ' + port + ' 750 '
    return subprocess.call(cmd, shell=True)


def upload_firmware(firmware):
    cmd = 'dfu-util -d 1eaf:0003 -a 2 -D ' + firmware + ' -R'
    return subprocess.call(cmd, shell=True)


parser = argparse.ArgumentParser(description='Process and check the logs.')
parser.add_argument('-d', '--dfu', action='store_true', help='Module is already in DFU mode',required=False)
parser.add_argument('-f', '--firmware', help='Firmware *.bin file path', type=str, default='Multiprotocol.ino.bin')
parser.add_argument('-p', '--port', type=str, default='')
args = parser.parse_args()

if not args.dfu:
    spinner = spinning_cursor()
    port = args.port
    if port == '':
        if not dev_not_exists('/dev/pats_mm1'):
            port = '/dev/pats_mm1'
        else:
            port = '/dev/pats_mm0'

    while dev_not_exists(port):
        c = next(spinner)
        s = c + 'Waiting for port ' + port + '...' + c
        print(s, end='')
        sys.stdout.flush()
        time.sleep(0.1)
        for x in s:
            sys.stdout.write('\b')

    print('Port detected: ' + port + '. Resetting to DFU mode...')
    time.sleep(0.5)
    reset_to_dfu(port)
print('Uploading ' + args.firmware)
time.sleep(1)
if upload_firmware(args.firmware):
    print("Error, dfu upload failed")
    exit(1)

print('\nAll done!')
