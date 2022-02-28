#!/usr/bin/env python3
import os
import time
import struct
import threading
import argparse
import serial
import lib_base as lb
import lib_serialization as ls


parser = argparse.ArgumentParser(description='Baseboard configure tool')
parser.add_argument('-c', help="Externally measured voltage to which to calibrate", required=False)
args = parser.parse_args()


if os.path.exists(lb.disable_baseboard_flag):
    print('Baeboard disable flag detected. Waiting...')
    while os.path.exists(lb.disable_baseboard_flag):
        time.sleep(1)
if not os.path.exists('/dev/baseboard'):
    print('No baseboard detected. Waiting...')
    while not os.path.exists('/dev/baseboard'):
        time.sleep(1)
if os.path.exists(lb.disable_charging_flag):
    print('Charging disable flag detected. Waiting...')
    while os.path.exists(lb.disable_charging_flag):
        time.sleep(1)


print('Connecting baseboard...')
comm = serial.Serial('/dev/baseboard', 115200, timeout=1)
print('Connected to baseboard')

exit_now = False


def read_do_work():
    dummy_pkg = ls.SerialBaseboard2NUCPackage()
    serial_data = bytearray()
    while not exit_now:
        c = comm.read(1)
        serial_data += c
        if len(serial_data) > 5 * struct.calcsize(dummy_pkg.format):
            print("Receiving serial data garbage...")
            serial_data.clear()

        if c:
            if ord(c) == ord(dummy_pkg.ender) and len(serial_data) >= struct.calcsize(dummy_pkg.format):
                pkg_start = len(serial_data) - struct.calcsize(dummy_pkg.format)
                if serial_data[pkg_start] == ord(dummy_pkg.pre_header):
                    new_pkg = ls.SerialBaseboard2NUCPackage()
                    new_pkg.parse(serial_data[pkg_start:])

                    if pkg_start > 0:
                        try:
                            print("Serial received: " + serial_data[:pkg_start].decode('utf-8'))
                        except Exception as e:  # pylint: disable=broad-except
                            print('Received corrupted data from baseboard :( ' + str(e))
                    if new_pkg.firmware_version == dummy_pkg.firmware_version:
                        print(str(round(new_pkg.up_duration / 1000, 2)) + 's: ' + ls.charging_state_names[new_pkg.charging_state] + ' ' + str("%.2f" % round(new_pkg.mah_charged, 2)) + 'mah in ' + str("%.2f" % round(new_pkg.charging_duration / 1000, 2)) + 's ' + str("%.2f" % round(new_pkg.charging_amps, 2)) + 'A / ' + str("%.2f" % round(new_pkg.setpoint_amps, 2)) + 'A ' + str("%.2f" % round(new_pkg.charging_volts, 2)) + 'v bat: ' + str("%.2f" % round(new_pkg.battery_volts, 2)) + 'v ' + str("%.2f" % round(new_pkg.charge_resistance, 2)) + 'Ω. Drone: ' + str("%.2f" % round(new_pkg.drone_amps_burn, 2)) + ' A. PWM: ' + str(new_pkg.charging_pwm) + ' fan: ' + str(new_pkg.measured_fan_speed) + ' wdt: ' + str(new_pkg.watchdog_state))

                        if len(serial_data) > struct.calcsize(dummy_pkg.format):
                            try:
                                print(serial_data[0:pkg_start].decode('utf-8'))
                            except Exception as e:  # pylint: disable=broad-except
                                print("Error weird serial bytes: " + str(e))
                        serial_data.clear()
                    else:
                        print('pkg version mismatch')


read_thread = threading.Thread(target=read_do_work)
read_thread.start()

time.sleep(3)  # allow the baseboard to boot before sending commands:


if args.c:
    chrg_pkg = ls.SerialNUC2BaseboardChargingPackage()
    chrg_pkg.enable_charging = 1
    chrg_pkg.reset_calibration = 1
    comm.write(chrg_pkg.pack())
    print("Calibration reset. Calibrating...")
    time.sleep(2)
    chrg_pkg.calibrate = 1
    chrg_pkg.volts = float(args.c)
    print("Setting calibration")
    comm.write(chrg_pkg.pack())
    time.sleep(1)
    chrg_pkg.calibrate = 0
    print("Calibrated")
    comm.write(chrg_pkg.pack())

    time.sleep(3)
    exit_now = True
    read_thread.join()
