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
import lib_serialization as ls


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

Path(lb.socket_dir).mkdir(parents=True, exist_ok=True)
logger.info('Starting baseboard!')


def led():
    dt_last_executor_msg = (datetime.now() - executor.last_msg_time).total_seconds()
    if dt_last_executor_msg > 20:
        rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_internal_system_error.value[0]
    rgb_led_pkg.internet_OK = daemon_pkg.internet_OK
    if os.path.exists(lb.disable_tunnel_flag):
        rgb_led_pkg.internet_OK = 1
    dt_last_deamon_msg = (datetime.now() - daemon.last_msg_time).total_seconds()
    if new_pkg.up_duration > prev_pkg.up_duration and dt_last_deamon_msg < 60:
        wdt_pkg = ls.SerialNUC2BaseboardWatchdogPackage()
        if not os.path.exists(lb.disable_watchdog_flag):
            wdt_pkg.watchdog_enabled = 1
        comm.write(wdt_pkg.pack())

        rgb_led_pkg.daemon_OK = dt_last_deamon_msg < 10
        rgb_led_pkg.post_processing = daemon_pkg.post_processing + daemon_pkg.rendering
    else:
        logger.warning('Uh oh. Nothing received from daemon.py. Do we need the hardware watchdog to hard reboot this thing?')
        rgb_led_pkg.daemon_OK = 0
        rgb_led_pkg.post_processing = 0
    comm.write(rgb_led_pkg.pack())


def executor_receiver(msg):
    # print("Received from exe: " + msg.decode(sys.stdout.encoding))
    while len(msg) > 3:
        if msg[0] == ord(ls.EXECUTOR_PACKAGE_PRE_HEADER):
            if msg[3] == ord(ls.baseboard_package_headers.header_SerialExecutor2BaseboardAllowChargingPackage.value[0]):
                comm.write(msg[:struct.calcsize(ls.SerialExecutor2BaseboardAllowChargingPackage().format)])
                msg = msg[struct.calcsize(ls.SerialExecutor2BaseboardAllowChargingPackage().format):]
            elif msg[1] == ord(ls.executor_package_headers.header_SocketExecutorStatePackage.value[0]):
                executor_state_pkg.parse(msg[:struct.calcsize(executor_state_pkg.format)])
                msg = msg[struct.calcsize(executor_state_pkg.format):]
                if executor_state_pkg.executor_state == ls.executor_states.es_brightness_restart.value[0] or executor_state_pkg.executor_state == ls.executor_states.es_wait_for_darkness.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_wait_for_darkness.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_wait_for_plukker.value[0] or executor_state_pkg.executor_state == ls.executor_states.es_plukker_restart.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_wait_for_plukkers.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_pats_c.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_c_OK.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_pats_x.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_x_OK.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_realsense_reset.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_realsense_reset.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_init.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_starting.value[0]\
                        or executor_state_pkg.executor_state == ls.executor_states.es_hardware_check.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_realsense_init.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_init_vision.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_closing.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_periodic_restart.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_user_restart.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_drone_config_restart.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_executor_start.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_watchdog_restart.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_wait_for_angle.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_xml_config_problem.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_realsense_not_found.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_drone_version_mismatch.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_realsense_fps_problem.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_realsense_frame_loss_problem.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_rc_problem.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_baseboard_problem.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_realsense_error.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_daemon_problen.value[0] \
                        or executor_state_pkg.executor_state == ls.executor_states.es_runtime_error.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_internal_system_error.value[0]
                print('rgb_led_pkg.led1state: ' + str(rgb_led_pkg.led1state))
            else:
                msg = ''
                logger.warning('Weird package received from executor...')


def deamon_receiver(msg):
    if msg[0] == ord(ls.BASEBOARD_PACKAGE_PRE_HEADER):
        if msg[3] == ord(ls.baseboard_package_headers.header_SocketDaemonLink2BaseboardLinkPackage.value[0]):
            daemon_pkg.parse(msg)


executor_state_pkg = ls.SocketExecutorStatePackage()
daemon_pkg = ls.SocketDaemon2BaseboardLinkPackage()
daemon = socket_communication('Daemon', lb.socket_baseboard2daemon, True, deamon_receiver)
executor = socket_communication('Executor', lb.socket_baseboard2executor, True, executor_receiver)

while True:
    try:
        try:
            logger.info('Connecting baseboard...')
            comm = serial.Serial('/dev/baseboard', 115200)
        except Exception as e:  # pylint: disable=broad-except
            logger.error('Connection to baseboard failed: ' + str(e))
            time.sleep(10)
            continue
        logger.info('Connected to baseboard')

        time.sleep(2)  # allow the baseboard to boot before sending commands:
        if not os.path.exists(lb.disable_watchdog_flag):
            wdt_pkg = ls.SerialNUC2BaseboardWatchdogPackage()
            wdt_pkg.watchdog_enabled = 1
            comm.write(wdt_pkg.pack())
            logger.info('Watchdog enabled')
        else:
            wdt_pkg = ls.SerialNUC2BaseboardWatchdogPackage()
            wdt_pkg.watchdog_enabled = 0
            comm.write(wdt_pkg.pack())
            logger.info('Watchdog DISABLED')
        if not os.path.exists(lb.disable_charging_flag):
            chrg_pkg = ls.SerialNUC2BaseboardChargingPackage()
            chrg_pkg.enable_charging = 1
            comm.write(chrg_pkg.pack())
            logger.info('Charging enabled')
        else:
            comm.write(ls.SerialNUC2BaseboardChargingPackage().pack())
            logger.info('Charging DISABLED')
        if not os.path.exists(lb.disable_ir_led_flag):
            led_pkg = ls.SerialNUC2BaseboardLedPowerPackage()
            led_pkg.led_power = 1
            comm.write(led_pkg.pack())
            logger.info('IR LED enabled')
        else:
            comm.write(ls.SerialNUC2BaseboardLedPowerPackage().pack())
            logger.info('IR LED DISABLED')
        if not os.path.exists(lb.disable_fan_flag):
            fan_pkg = ls.SerialNUC2BaseboardFanPackage()
            fan_pkg.fan_pwm = 1
            comm.write(fan_pkg.pack())
            logger.info('Fan enabled')
        else:
            comm.write(ls.SerialNUC2BaseboardFanPackage().pack())
            logger.info('Fan DISABLED')

        prev_pkg = ls.SerialBaseboard2NUCPackage()

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
        first_pkg = True
        rgb_led_pkg = ls.SerialNUC2BaseboardRGBLEDPackage()
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

            if ord(c) == ord(prev_pkg.ender) and len(serial_data) >= struct.calcsize(prev_pkg.format):
                pkg_start = len(serial_data) - struct.calcsize(prev_pkg.format)
                if serial_data[pkg_start] == ord(prev_pkg.pre_header):
                    new_pkg = ls.SerialBaseboard2NUCPackage()
                    new_pkg.parse(serial_data[pkg_start:])

                    if pkg_start > 0:
                        try:
                            logger.info("Serial received: " + serial_data[:pkg_start].decode('utf-8'))
                        except Exception as e:  # pylint: disable=broad-except
                            logger.info('Received corrupted data from baseboard :( ' + str(e))
                    if new_pkg.firmware_version == prev_pkg.firmware_version:
                        led()

                        if first_pkg:
                            first_pkg = False
                            logger.debug('# boots: ' + str(new_pkg.baseboard_boot_count) + ', # wdt boots:' + str(new_pkg.watchdog_boot_count))
                        logger.debug(str(round(new_pkg.up_duration / 1000, 2)) + 's: ' + charging_state_names[new_pkg.charging_state] + ' ' + str("%.2f" % round(new_pkg.mah_charged, 2)) + 'mah in ' + str("%.2f" % round(new_pkg.charging_duration / 1000, 2)) + 's ' + str("%.2f" % round(new_pkg.charging_amps, 2)) + 'A / ' + str("%.2f" % round(new_pkg.setpoint_amp, 2)) + 'A ' + str("%.2f" % round(new_pkg.charging_volts, 2)) + 'v bat: ' + str("%.2f" % round(new_pkg.battery_volts, 2)) + 'v ' + str("%.2f" % round(new_pkg.charge_resistance, 2)) + 'Ω. Drone: ' + str("%.2f" % round(new_pkg.drone_amps_burn, 2)) + ' A. PWM: ' + str(new_pkg.charging_pwm) + ' fan: ' + str(new_pkg.measured_fan_speed) + ' wdt: ' + str(new_pkg.watchdog_state) + ' exe: ' + str(executor_state_pkg.executor_state))
                        executor_pkg = serial_data[pkg_start:]
                        executor.send(executor_pkg)
                        prev_pkg = new_pkg
                        serial_data.clear()
                    else:
                        logger.warning('pkg version mismatch')
    except Exception as e:  # pylint: disable=broad-except
        logger.error('Baseboard: ' + str(e))
