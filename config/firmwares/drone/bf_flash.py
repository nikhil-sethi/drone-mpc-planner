#!/usr/bin/env python3
import sys,os,grp,subprocess,stat,argparse,time
import serial # pip3 install pyserial or if you use conda: conda install -c anaconda pyserial


def dev_not_exists(path):

	try:
		if not stat.S_ISBLK(os.stat(path).st_mode):
			stat_info = os.stat(path)
			gid = stat_info.st_gid
			group = grp.getgrgid(gid)[0]
			return group != 'dialout'
		return True

	except:
		return True

def spinning_cursor():
    while True:
        for cursor in '|/-\\':
            yield cursor

def reset_to_dfu(port):
	ser = serial.Serial(port=port)
	time.sleep(0.1)
	ser.write('#'.encode())
	time.sleep(1)
	ser.write('bl\n'.encode())

def upload_firmware(firmware):
	cmd = 'dfu-util -s 0x08000000 -a 0 -R -D ' + firmware
	return subprocess.call(cmd,shell=True)


def upload_settings(port,tx_id,rx_num,settings):
	ser = serial.Serial(port=port)

	ser.write('#'.encode())
	time.sleep(1)

	with open(settings, 'r') as fp:
		for cnt, line in enumerate(fp):
			print("Line {}: {}".format(cnt, line), end = '')

			if 'frsky_spi_tx_id' in line:
				line = 'set frsky_spi_tx_id = ' + str(tx_id) + ',0\n'
			if 'frsky_x_rx_num' in line:
				line = 'set frsky_x_rx_num = ' + str(rx_num) + ',0\n'

			ser.write(line.encode())
			ser.write('\n'.encode())
			time.sleep(0.01)
		ser.write('save\n'.encode())
		time.sleep(0.5)
		ser.close()
	ser = serial.Serial(port=port)
	time.sleep(1)

	print("Uploading settings successfull")

parser = argparse.ArgumentParser(description='Process and check the logs.')
parser.add_argument('-i', '--drone-id',help='Drone id in case of using D16',type=str,default='1')
parser.add_argument('-j', '--tx-id',help='Drone id in case of using D8',type=str,default='3')
parser.add_argument('-f', '--firmware',help='Firmware *.bin file path',type=str,default='pats_4.2.bin')
parser.add_argument('-s', '--settings',help='Betaflight diff settings file path', type=str,default='BF_Anvil_4.2_3led.txt')
parser.add_argument('-p', '--port',type=str,default='/dev/ttyACM0')
parser.add_argument('-n', '--no-flashing', action='store_true')
parser.add_argument('-d', '--dfu', help='Assume drone is already in DFU mode', action='store_true')
args = parser.parse_args()

if int(args.drone_id) > 63:
	print("Error: drone-id > 63")
	exit(1)

spinner = spinning_cursor()

if args.no_flashing and args.dfu:
	print('Error: conflicting arguments!')
	exit(1)

if not args.no_flashing:
	if not args.dfu:
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

delay = False
while dev_not_exists(args.port):
	c =  next(spinner)
	if not args.no_flashing:
		s = c + 'Human, replug usb NOW' + c
	else:
		s = c + 'Waiting for port: ' + args.port + c
	print(s, end = '')
	sys.stdout.flush()
	time.sleep(0.1)
	for x in s:
		sys.stdout.write('\b')

print('Port detected. Uploading settings...')
upload_settings(args.port,args.tx_id,args.drone_id,args.settings)

print('\nAll done!')
