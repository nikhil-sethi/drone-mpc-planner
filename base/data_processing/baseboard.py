#!/usr/bin/env python3
from datetime import datetime
import sys
import os
import subprocess
import time
import struct
import logging
import logging.handlers
from pathlib import Path
import serial
from lib_socket import socket_communication
import lib_base as lb


class SerialPackage:

    format = '=cHBBLBfffffffBLHc'  # https://docs.python.org/3/library/struct.html?highlight=struct#format-characters
    header = 'P'
    version = 5,  # note: put this in __init__ as well. Because python bug
    led_state = 0
    watchdog_state = 0
    up_duration = 0
    charging_state = 0
    battery_voltage = 0
    smoothed_voltage = 0
    charging_current = 0
    desired_current = 0
    mah_charged = 0
    contact_resistance = 0
    drone_current_usage = 0
    charging_pwm = 0
    charging_duration = 0
    fan_speed = 0
    ender = '\n'

    def __init__(self):
        self.version = 5  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.header,
         self.version,
         self.led_state,
         self.watchdog_state,
         self.up_duration,
         self.charging_state,
         self.battery_voltage,
         self.smoothed_voltage,
         self.charging_current,
         self.desired_current,
         self.mah_charged,
         self.contact_resistance,
         self.drone_current_usage,
         self.charging_pwm,
         self.charging_duration,
         self.fan_speed,
         self.ender,
         ) = fields


start_sha = subprocess.check_output(["git", "describe"]).decode(sys.stdout.encoding).strip()
logging.basicConfig()
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger('baseboard')
fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + 'baseboard.log', maxBytes=1024 * 1024 * 100, backupCount=1)
fh.setFormatter(file_format)
error_file_handler = logging.handlers.WatchedFileHandler(filename=lb.daily_errs_log)  # a watched file handler makes sure that logging continues to the new file if it is rotated.
error_file_handler.setFormatter(file_format)
error_file_handler.level = logging.ERROR
logger.addHandler(fh)
logger.addHandler(error_file_handler)
logger.setLevel(logging.DEBUG)
logger.info('Baseboard reporting in!')
logger.info('sha: ' + start_sha)

if os.path.exists(lb.disable_baseboard_flag):
    logger.info('System does not have a basboard...')
    while os.path.exists(lb.disable_baseboard_flag):
        time.sleep(10)
elif not os.path.exists('/dev/baseboard'):
    logger.error('No baseboard connected, add disable_baseboard flag.')
    while not os.path.exists('/dev/baseboard'):
        time.sleep(10)

Path(lb.socket_dir).mkdir(parents=True, exist_ok=True)
logger.info('Starting baseboard!')

daemon = socket_communication('Daemon', lb.socket_baseboard2daemon, True)
pats = socket_communication('Pats', lb.socket_baseboard2pats, True)


while True:
    try:
        logger.info('Connecting baseboard...')
        comm = serial.Serial('/dev/baseboard', 115200, timeout=1)
        logger.info('Connected to baseboard')

        time.sleep(2)  # allow the baseboard to boot before sending commands:
        if not os.path.exists(lb.disable_watchdog_flag):
            comm.write(b'enable watchdog\n')
            logger.info('Watchdog enabled')
        else:
            comm.write(b'disable watchdog\n')
            logger.info('Watchdog DISABLED')
        if not os.path.exists(lb.disable_charging_flag):
            comm.write(b'enable charger\n')
            logger.info('Charging enabled')
        else:
            comm.write(b'disable charger\n')
            logger.info('Charging DISABLED')

        prev_pkg = SerialPackage()

        charging_state_names = [
            'disabled',
            'init',
            'drone_not_on_pad',
            'contact_problem',
            'bat_dead',
            'bat_does_not_charge',
            'revive_charging',
            'normal_charging',
            'trickle_charging',
            'discharge',
            'measure',
            'calibratin']

        prev_byte = ''
        serial_data = bytearray()
        prev_sha_check_time = datetime.now()
        while True:

            if (datetime.now() - prev_sha_check_time).seconds > 10:
                prev_sha_check_time = datetime.now()
                current_sha = subprocess.check_output(["git", "describe"]).decode(sys.stdout.encoding).strip()
                if start_sha != current_sha:
                    print("SHA change detected. Restarting!")
                    print("Closing serial...")
                    comm.close()
                    print("Closing pats socket...")
                    pats.close()
                    print("Closing daemon socket...")
                    daemon.close()
                    exit(0)

            c = comm.read(1)
            serial_data += c
            if len(serial_data) > 5 * struct.calcsize(prev_pkg.format):
                logger.warning("Receiving serial data garbage...")
                serial_data.clear()

            if c:
                if ord(c) == ord(prev_pkg.ender) and len(serial_data) >= struct.calcsize(prev_pkg.format):
                    pkg_start = len(serial_data) - struct.calcsize(prev_pkg.format)
                    if serial_data[pkg_start] == ord(prev_pkg.header):
                        new_pkg = SerialPackage()
                        new_pkg.parse(serial_data[pkg_start:])

                        if pkg_start > 0:
                            try:
                                logger.info(serial_data[:pkg_start].decode('utf-8'))
                            except Exception as e:  # pylint: disable=broad-except
                                logger.info('Received corrupted data from baseboard :( ' + str(e))
                        if new_pkg.version == prev_pkg.version:

                            dt = (datetime.now() - daemon.last_msg_time).total_seconds()
                            if new_pkg.up_duration > prev_pkg.up_duration and dt < 600:
                                comm.write(b"Harrow!\n")
                            else:
                                logger.warning('Uh oh. Nothing received from daemon.py. Do we need the hardware watchdog to hard reboot this thing?')

                            logger.debug(str(round(new_pkg.up_duration / 1000, 2)) + 's: ' + charging_state_names[new_pkg.charging_state] + ' ' + str("%.2f" % round(new_pkg.mah_charged, 2)) + 'mah in ' + str("%.2f" % round(new_pkg.charging_duration / 1000, 2)) + 's ' + str("%.2f" % round(new_pkg.charging_current, 2)) + 'A / ' + str("%.2f" % round(new_pkg.desired_current, 2)) + 'A ' + str("%.2f" % round(new_pkg.smoothed_voltage, 2)) + 'v bat: ' + str("%.2f" % round(new_pkg.battery_voltage, 2)) + 'v ' + str("%.2f" % round(new_pkg.contact_resistance, 2)) + 'Ω. Drone: ' + str("%.2f" % round(new_pkg.drone_current_usage, 2)) + ' A. PWM: ' + str(new_pkg.charging_pwm) + ' fan: ' + str(new_pkg.fan_speed))
                            pats_pkg = serial_data[pkg_start:]
                            pats.send(pats_pkg)
                            prev_pkg = new_pkg
                            serial_data.clear()
                        else:
                            logger.warning('pkg version mismatch')
    except Exception as e:  # pylint: disable=broad-except
        logger.error('Baseboard: ' + str(e))
        time.sleep(10)
        if 'readiness' in str(e) or 'could not open port /dev/baseboard' in str(e):
            logger.info('Try to reset Baseboard.')
            if time.monotonic() > 3600 * 24:  # time.monotonic() is uptime in seconds
                cmd = 'sudo rtcwake -m off -s 120'
                lb.execute(cmd, 1, logger_name='baseboard')
            else:
                logger.info('Last reboot was less than a day ago, so waiting for hardware watchdog...')
                time.sleep(60)
