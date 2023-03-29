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

start_sha = subprocess.check_output(["git", "rev-parse", "HEAD"]).decode(sys.stdout.encoding).strip()
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
start_time = datetime.today().strftime("%d-%m-%Y %H:%M:%S")
logger.info('Baseboard reporting in!')
print(start_time)
logger.info('sha: ' + start_sha)

if os.path.exists(lb.disable_baseboard_flag):
    logger.info('Baseboard disabled by flag: ' + lb.disable_baseboard_flag)
    while os.path.exists(lb.disable_baseboard_flag):
        time.sleep(10)
elif not os.path.exists('/dev/baseboard'):
    logger.error('No baseboard connected! Add the disable_baseboard flag to work around this issue.')
    while not os.path.exists('/dev/baseboard'):
        time.sleep(10)

Path(lb.socket_dir).mkdir(parents=True, exist_ok=True)
logger.info('Starting baseboard!')


def led():
    if not os.path.exists(lb.disable_executor_flag):
        dt_last_executor_msg = (datetime.now() - executor_comm.last_msg_time).total_seconds()
    else:
        dt_last_executor_msg = 0
        rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_blind_OK.value[0]
    if dt_last_executor_msg > 20:
        rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_executor_problem.value[0]
    rgb_led_pkg.internet_OK = daemon_pkg.internet_OK
    if os.path.exists(lb.disable_tunnel_flag):
        rgb_led_pkg.internet_OK = 1
    dt_last_deamon_msg = (datetime.now() - daemon_comm.last_msg_time).total_seconds()
    if new_pkg.up_duration > prev_pkg.up_duration and dt_last_deamon_msg < 60:
        wdt_pkg = ls.SerialNUC2BaseboardWatchdogPackage()
        if not os.path.exists(lb.disable_watchdog_flag):
            wdt_pkg.watchdog_enabled = 1
        comm.write(wdt_pkg.pack())

        rgb_led_pkg.daemon_OK = dt_last_deamon_msg < 10 or os.path.exists(lb.disable_daemonlink_flag)
        rgb_led_pkg.post_processing = daemon_pkg.post_processing or daemon_pkg.rendering
    elif not os.path.exists(lb.disable_daemonlink_flag):
        logger.warning('Uh oh. Nothing received from daemon.py. Do we need the hardware watchdog to hard reboot this thing?')
        rgb_led_pkg.daemon_OK = 0
        rgb_led_pkg.post_processing = 0
    comm.write(rgb_led_pkg.pack())


def executor_receiver(msg):
    # print("Received from exe: " + msg.decode(sys.stdout.encoding))
    while len(msg) > 3:
        if msg[0] == ord(ls.EXECUTOR_PACKAGE_PRE_HEADER):
            if msg[3] == ord(ls.baseboard_package_headers.header_SerialExecutor2BaseboardAllowChargingPackage.value[0]):
                # allow_charging_pkg = ls.SerialExecutor2BaseboardAllowChargingPackage()
                # allow_charging_pkg.parse(msg[:struct.calcsize(allow_charging_pkg.format)])
                # comm.write(allow_charging_pkg.pack())
                comm.write(msg[:struct.calcsize(ls.SerialExecutor2BaseboardAllowChargingPackage().format)])
                msg = msg[struct.calcsize(ls.SerialExecutor2BaseboardAllowChargingPackage().format):]
            elif msg[1] == ord(ls.executor_package_headers.header_SocketExecutorStatePackage.value[0]):
                executor_state_pkg.parse(msg[:struct.calcsize(executor_state_pkg.format)])
                msg = msg[struct.calcsize(executor_state_pkg.format):]
                rgb_led_pkg.drone_issues = executor_state_pkg.drone_issue
                rgb_led_pkg.light_level = executor_state_pkg.light_level
                if executor_state_pkg.executor_state == ls.executor_states.es_light_level_restart.value[0] or executor_state_pkg.executor_state == ls.executor_states.es_wait_for_light_level.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_wait_for_light_level.value[0]
                elif executor_state_pkg.executor_state == ls.executor_states.es_enable_window_restart.value[0] or executor_state_pkg.executor_state == ls.executor_states.es_wait_for_enable_window.value[0]:
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_wait_for_enable_window.value[0]
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
                        or executor_state_pkg.executor_state == ls.executor_states.es_wait_for_cam_angle.value[0] \
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
                    rgb_led_pkg.led1state = ls.rgb_led_1_states.LED1_executor_problem.value[0]
            else:
                msg = ''
                logger.warning('Weird package received from executor...')
        else:
            msg = msg[1:]
            logger.warning('Some weird byte was received from executor...')


def deamon_receiver(msg):
    if msg[0] == ord(ls.BASEBOARD_PACKAGE_PRE_HEADER):
        if msg[3] == ord(ls.baseboard_package_headers.header_SocketDaemonLink2BaseboardLinkPackage.value[0]):
            daemon_pkg.parse(msg)
    if not daemon_pkg.internet_OK:
        logger.warning('Internet is not OK...')


rgb_led_pkg = ls.SerialNUC2BaseboardRGBLEDPackage()  # must be initialized before starting the socket comm
executor_state_pkg = ls.SocketExecutorStatePackage()
daemon_pkg = ls.SocketDaemon2BaseboardLinkPackage()
daemon_comm = socket_communication('Daemon', 'baseboard', lb.socket_baseboard2daemon, True, deamon_receiver)
if not os.path.exists(lb.disable_executor_flag):
    executor_comm = socket_communication('Executor', 'baseboard', lb.socket_baseboard2executor, True, executor_receiver)

while True:
    try:
        logger.info('Connecting baseboard...')
        comm = serial.Serial('/dev/baseboard', 115200, timeout=1)
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

        prev_byte = ''
        serial_data = bytearray()
        prev_sha_check_time = datetime.now()
        first_pkg = True
        rgb_led_pkg = ls.SerialNUC2BaseboardRGBLEDPackage()
        while True:

            if (datetime.now() - prev_sha_check_time).seconds > 10:
                prev_sha_check_time = datetime.now()
                current_sha = subprocess.check_output(["git", "rev-parse", "HEAD"]).decode(sys.stdout.encoding).strip()
                if start_sha != current_sha:
                    logger.warning("SHA discrepancy detected!")

            c = comm.read(1)
            serial_data += c
            if len(serial_data) > 5 * struct.calcsize(prev_pkg.format):
                logger.warning("Receiving serial data garbage...")
                serial_data.clear()

            if c:
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
                            logger.debug(str("%.1f" % round(new_pkg.up_duration / 1000, 2)) + 's: '
                                         + ls.charging_state_names[new_pkg.charging_state]
                                         + str("%.2f" % round(new_pkg.charging_amps, 2)) + 'A / '
                                         + str("%.2f" % round(new_pkg.setpoint_amps, 2)) + 'A, PWM:'
                                         + str("%3d" % new_pkg.charging_pwm) + '\tBat: ' + str("%.2f" % round(new_pkg.battery_volts, 2))
                                         + 'v Chrg: ' + str("%.2f" % round(new_pkg.charging_volts, 1))
                                         + 'v Gnd: ' + str("%.2f" % round(new_pkg.ground_volts, 1))
                                         + 'v ' + ls.led0_state_names[new_pkg.led0]
                                         + ' ' + ls.led1_state_names[new_pkg.led1]
                                         + ' \tDrone: ' + str("%.2f" % round(new_pkg.drone_amps_burn, 2)) + ' A, '
                                         + str("%.2f" % round(new_pkg.mah_charged, 2)) + 'mah in ' + str("%.0f" % round(new_pkg.charging_duration / 1000, 2)) + 's'
                                         + '\tfan: ' + str(new_pkg.measured_fan_speed)
                                         + ' wdt: ' + str(bool(new_pkg.watchdog_state))
                                         )
                            executor_pkg = serial_data[pkg_start:]
                            if not os.path.exists(lb.disable_executor_flag):
                                if not executor_comm.connection_ok:
                                    logger.info('Executor link LOST since: ' + executor_comm.connection_lost_datetime.strftime("%d-%m-%Y %H:%M:%S"))
                                executor_comm.send(executor_pkg)  # force send thread to find out connection is lost
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
