#!/usr/bin/env python3
import serial
import time
import signal
import sys
import argparse

global ser
ser = 0

def start_serial():
    try:
        global ser
        ser = serial.Serial('/dev/baseboard', 115200, timeout=0.01)
    except:
        print('device not found')

global watchdog_demo
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    global watchdog_demo
    if (watchdog_demo):
        try_write(b'disable watchdog')
    ser.close()
    print("exit")
    sys.exit(-1)


signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(description='Test serial communication with baseboard')
parser.add_argument('-l', '--led-demo',help='Turn leds on and off with 3 second intervals',action='store_true')
parser.add_argument('-r', '--reboot',help='Reboots the nuc every 15 seconds',action='store_true')
parser.add_argument('-w', '--watchdog',help='Enables watchdog and disables on exit',action='store_true')
parser.add_argument('-a', '--alive',help='Sends stay alive message every 15 seconds',action='store_true')
parser.add_argument('-e', '--enable-watchdog',help='Enables watchdog and exits',action='store_true')
parser.add_argument('-d', '--disable-watchdog',help='Disables watchdog and exits',action='store_true')
parser.add_argument('-o', '--output',help='Shows output',action='store_true')
args = parser.parse_args()

led_demo = args.led_demo
reboot_demo = args.reboot
watchdog_demo = args.watchdog
enable_watchdog = args.enable_watchdog
disable_watchdog = args.disable_watchdog
send_alive = args.alive
show_output = args.output

led_is_on = True
watchdog_is_on = True
last_led_time = time.time()
last_reboot_time = time.time()
last_watchdog_time = time.time()
start_serial()

def try_write(line):
    try:
        print(line)
        ser.write(line)
    except:
        print('write failure')
        time.sleep(1)

while True:
    if (not ser):
        start_serial()
        time.sleep(1)
    line = ""
    try:
        line = ser.readline()
    except:
        print('read failure')
        time.sleep(1)

    if len(line) and show_output:
        print(line)

    if (led_demo):
        if (time.time() - last_led_time > 3):
            if (led_is_on):
                try_write(b"disable led")
            else:
                try_write(b"enable led")
            led_is_on = not led_is_on
            last_led_time = time.time()

    if (reboot_demo):
        if (time.time() - last_reboot_time > 15):
            try_write(b"reboot nuc")
            last_reboot_time = time.time()

    if (enable_watchdog):
        try_write(b"enable watchdog")
        quit()

    if (disable_watchdog):
        try_write(b"disable watchdog")
        quit()

    if (watchdog_demo):
        if (not watchdog_is_on):
            try_write(b"enable watchdog")
            watchdog_is_on = True

    if (send_alive):
        if (time.time() - last_watchdog_time > 10):
            try_write(b"stay alive") # or whatever
            last_watchdog_time = time.time()
