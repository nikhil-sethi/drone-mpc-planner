#!/usr/bin/env python3
from datetime import datetime
import sys
import os
import subprocess
import time
import struct
from enum import Enum
import logging
import logging.handlers
from pathlib import Path
import serial
from lib_socket import socket_communication
import lib_base as lb

BASEBOARD_FIRMWARE_VERSION = 6
PACKAGE_PRE_HEADER = 'B'


class baseboard_package_headers(Enum):
    header_SerialBaseboard2NUCPackage = 'P',
    header_SerialNUC2BaseboardChargingPackage = 'C',
    header_SerialNUC2BaseboardLedPowerPackage = 'L',
    header_SerialNUC2BaseboardWatchdogPackage = 'W',
    header_SerialNUC2BaseboardFanPackage = 'F',
    header_SerialNUC2BaseboardNUCResetPackage = 'N',
    header_SerialNUC2BaseboardEEPROMPackage = 'E',
    header_SerialExecutor2BaseboardAllowChargingPackage = 'S',


class SerialBaseboard2NUCPackage:

    format = '=cHcBBLBfffffffBLHc'  # https://docs.python.org/3/library/struct.html?highlight=struct#format-characters
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialBaseboard2NUCPackage
    led_state = 0
    watchdog_state = 0
    up_duration = 0
    charging_state = 0
    battery_volts = 0
    charging_volts = 0
    charging_amps = 0
    setpoint_amp = 0
    mah_charged = 0
    charge_resistance = 0
    drone_amps_burn = 0
    charging_pwm = 0
    charging_duration = 0
    measured_fan_speed = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.led_state,
         self.watchdog_state,
         self.up_duration,
         self.charging_state,
         self.battery_volts,
         self.charging_volts,
         self.charging_amps,
         self.setpoint_amp,
         self.mah_charged,
         self.charge_resistance,
         self.drone_amps_burn,
         self.charging_pwm,
         self.charging_duration,
         self.measured_fan_speed,
         self.ender,
         ) = fields


class SerialNUC2BaseboardEEPROMPackage:

    format = '=cHcBBBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardEEPROMPackage.value[0]
    clear_config_all = 0,
    clear_config_hard = 0,
    clear_config_log = 0,
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.clear_config_all,
         self.clear_config_hard,
         self.clear_config_log,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.clear_config_all,
                           self.clear_config_hard,
                           self.clear_config_log,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardChargingPackage:

    format = '=cHcBBfBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardChargingPackage.value[0]
    enable_charging = 0
    calibrate = 0
    volts = 0
    reset_calibration = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.enable_charging,
         self.calibrate,
         self.volts,
         self.reset_calibration,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.enable_charging,
                           self.calibrate,
                           self.volts,
                           self.reset_calibration,
                           bytes(self.ender, 'utf-8')
                           )


class SerialExecutor2BaseboardAllowChargingPackage:

    format = '=cHcBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialExecutor2BaseboardAllowChargingPackage.value[0]
    allow_charging = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.allow_charging,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.allow_charging,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardLedPowerPackage:

    format = '=cHcBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardLedPowerPackage.value[0]
    led_power = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.led_power,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.led_power,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardWatchdogPackage:

    format = '=cHcBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardWatchdogPackage.value[0]
    watchdog_enabled = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
            self.version,
            self.header,
            self.watchdog_enabled,
            self.ender
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.watchdog_enabled,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardFanPackage:

    format = '=cHcBc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardFanPackage.value[0]
    fan_pwm = 0
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.fan_pwm,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           self.fan_pwm,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardNUCResetPackage:

    format = '=cHcc'
    pre_header = PACKAGE_PRE_HEADER
    version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardNUCResetPackage.value[0]
    ender = '\n'

    def __init__(self):
        self.version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.version,
         self.header,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.version),
                           bytes(self.header, 'utf-8'),
                           bytes(self.ender, 'utf-8')
                           )


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


def executor_receiver(msg):
    # print("Received from exe: " + msg.decode(sys.stdout.encoding) + '   allow: ' + str(msg[4]))
    comm.write(msg)


daemon = socket_communication('Daemon', lb.socket_baseboard2daemon, True)
executor = socket_communication('Executor', lb.socket_baseboard2executor, True, executor_receiver)

while True:
    try:
        logger.info('Connecting baseboard...')
        comm = serial.Serial('/dev/baseboard', 115200, timeout=1)
        logger.info('Connected to baseboard')

        time.sleep(2)  # allow the baseboard to boot before sending commands:
        if not os.path.exists(lb.disable_watchdog_flag):
            wdt_pkg = SerialNUC2BaseboardWatchdogPackage()
            wdt_pkg.watchdog_enabled = 1
            comm.write(wdt_pkg.pack())
            logger.info('Watchdog enabled')
        else:
            wdt_pkg = SerialNUC2BaseboardWatchdogPackage()
            wdt_pkg.watchdog_enabled = 0
            comm.write(wdt_pkg.pack())
            logger.info('Watchdog DISABLED')
        if not os.path.exists(lb.disable_charging_flag):
            chrg_pkg = SerialNUC2BaseboardChargingPackage()
            chrg_pkg.enable_charging = 1
            comm.write(chrg_pkg.pack())
            logger.info('Charging enabled')
        else:
            comm.write(SerialNUC2BaseboardChargingPackage().pack())
            logger.info('Charging DISABLED')
        if not os.path.exists(lb.disable_ir_led_flag):
            led_pkg = SerialNUC2BaseboardLedPowerPackage()
            led_pkg.led_power = 1
            comm.write(led_pkg.pack())
            logger.info('IR LED enabled')
        else:
            comm.write(SerialNUC2BaseboardLedPowerPackage().pack())
            logger.info('IR LED DISABLED')
        if not os.path.exists(lb.disable_fan_flag):
            fan_pkg = SerialNUC2BaseboardFanPackage()
            fan_pkg.fan_pwm = 1
            comm.write(fan_pkg.pack())
            logger.info('Fan enabled')
        else:
            comm.write(SerialNUC2BaseboardFanPackage().pack())
            logger.info('Fan DISABLED')

        prev_pkg = SerialBaseboard2NUCPackage()

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
            'wait_until_drone_ready',
            'calibrating'
        ]

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
                    print("Closing executor socket...")
                    executor.close()
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
                    if serial_data[pkg_start] == ord(prev_pkg.pre_header):
                        new_pkg = SerialBaseboard2NUCPackage()
                        new_pkg.parse(serial_data[pkg_start:])

                        if pkg_start > 0:
                            try:
                                logger.info("Serial received: " + serial_data[:pkg_start].decode('utf-8'))
                            except Exception as e:  # pylint: disable=broad-except
                                logger.info('Received corrupted data from baseboard :( ' + str(e))
                        if new_pkg.version == prev_pkg.version:

                            dt = (datetime.now() - daemon.last_msg_time).total_seconds()
                            if new_pkg.up_duration > prev_pkg.up_duration and dt < 600:
                                wdt_pkg = SerialNUC2BaseboardWatchdogPackage()
                                if not os.path.exists(lb.disable_watchdog_flag):
                                    wdt_pkg.watchdog_enabled = 1
                                comm.write(wdt_pkg.pack())
                            else:
                                logger.warning('Uh oh. Nothing received from daemon.py. Do we need the hardware watchdog to hard reboot this thing?')

                            logger.debug(str(round(new_pkg.up_duration / 1000, 2)) + 's: ' + charging_state_names[new_pkg.charging_state] + ' ' + str("%.2f" % round(new_pkg.mah_charged, 2)) + 'mah in ' + str("%.2f" % round(new_pkg.charging_duration / 1000, 2)) + 's ' + str("%.2f" % round(new_pkg.charging_amps, 2)) + 'A / ' + str("%.2f" % round(new_pkg.setpoint_amp, 2)) + 'A ' + str("%.2f" % round(new_pkg.charging_volts, 2)) + 'v bat: ' + str("%.2f" % round(new_pkg.battery_volts, 2)) + 'v ' + str("%.2f" % round(new_pkg.charge_resistance, 2)) + 'Ω. Drone: ' + str("%.2f" % round(new_pkg.drone_amps_burn, 2)) + ' A. PWM: ' + str(new_pkg.charging_pwm) + ' fan: ' + str(new_pkg.measured_fan_speed) + ' wdt: ' + str(new_pkg.watchdog_state))
                            executor_pkg = serial_data[pkg_start:]
                            executor.send(executor_pkg)
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
