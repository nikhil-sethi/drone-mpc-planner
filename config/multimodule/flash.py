#!/usr/bin/env python3
import sys,os,re,subprocess,stat,argparse,time
import serial # pip3 install pyserial or if you use conda: conda install -c anaconda pyserial

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
	subprocess.call(cmd,shell=True)
	cmd = './a.out ' + port + ' 750 '
	return subprocess.call(cmd,shell=True)

def upload_firmware(firmware):	
	cmd = 'dfu-util -d 1eaf:0003 -a 2 -D ' + firmware + ' -R'
	return subprocess.call(cmd,shell=True)


parser = argparse.ArgumentParser(description='Process and check the logs.')
parser.add_argument('-f', '--firmware',help='Firmware *.bin file path',type=str,default='Multiprotocol.ino.bin')
parser.add_argument('-p', '--port',type=str,default='/dev/pats_mm0')
args = parser.parse_args()

spinner = spinning_cursor()


while dev_not_exists(args.port):
	c =  next(spinner)
	s = c + 'Waiting for port ' + args.port + c
	print(s, end = '')
	sys.stdout.flush()
	time.sleep(0.1)
	for x in s:
		sys.stdout.write('\b')

print('Port detected. Resetting to DFU mode...')
time.sleep(0.5)
reset_to_dfu(args.port)
print('Uploading ' + args.firmware)
time.sleep(1)
if upload_firmware(args.firmware):
	print("Error, dfu upload failed")
	exit(1)

print('\nAll done!')
