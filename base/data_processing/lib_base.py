import logging
from pathlib import Path
import subprocess
import re
import sys
import os
import time
import socket
from datetime import datetime

flags_dir = os.path.expanduser('~/pats/flags/')
log_dir = os.path.expanduser('~/pats/logs/')
socket_dir = os.path.expanduser('~/pats/sockets/')
data_dir = os.path.expanduser('~/pats/data/')
json_dir = os.path.expanduser('~/pats/jsons/')
renders_dir = os.path.expanduser('~/pats/renders/')
executor_log_dir = os.path.expanduser('~/code/pats/base/build/logging/')

daily_errs_log = log_dir + 'all_errors.log'
term_log_path = log_dir + 'term.log'
disable_flag = flags_dir + 'disable'
cc_update_request = flags_dir + 'cc_update_request'
disable_tunnel_flag = flags_dir + 'disable_tunnel'
disable_daemonlink_flag = flags_dir + 'disable_daemonlink'
disable_baseboard_flag = flags_dir + 'disable_baseboard'
disable_charging_flag = flags_dir + 'disable_charging'
disable_watchdog_flag = flags_dir + 'disable_watchdog'
disable_executor_flag = flags_dir + 'disable_executor'
disable_ir_led_flag = flags_dir + 'disable_ir_led'
disable_fan_flag = flags_dir + 'disable_fan'
remain_inactive_flag = flags_dir + 'remain_inactive'
enable_wp_demo_flag = flags_dir + 'enable_wp_demo'

socket_baseboard2executor = socket_dir + 'baseboard2executor.socket'
socket_baseboard2daemon = socket_dir + 'baseboard2daemon.socket'
socket_executor2daemon = socket_dir + 'executor2daemon.socket'

local_status_txt_file = os.path.expanduser('~/pats/status/status.txt')
local_system_txt_file = os.path.expanduser('~/pats/status/system_info.txt')
local_status_im_file = os.path.expanduser('~/pats/status/monitor_tmp.jpg')
local_xml_folder = os.path.expanduser('~/code/pats/base/xml/')
local_pats_xml_override = os.path.expanduser('~/pats/pats.xml')

remote_status_txt_file = 'status/' + socket.gethostname() + '/status.txt'
remote_system_txt_file = 'status/' + socket.gethostname() + '/system.txt'
remote_status_im_file = 'status/' + socket.gethostname() + '/status.jpg'
remote_xml_folder = 'status/' + socket.gethostname() + '/'
remote_pats_xml_override = 'status/' + socket.gethostname() + '/pats_override.xml'


def str_to_datetime(string):
    return datetime.strptime(string, "%Y%m%d_%H%M%S")


def datetime_to_str(d: datetime):
    return d.strftime('%Y%m%d_%H%M%S')


def datetime_to_str_with_timezone(d: datetime):
    return d.astimezone().strftime('%Y%m%d_%H%M%S_%z')


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


def execute(cmd, retry=1, logger_name='', render_process_dir=None, verbose=True, raw_log_file=''):
    if logger_name != '':
        logger = logging.getLogger(logger_name)

    if len(raw_log_file):
        log = open(raw_log_file, "w", encoding="utf-8")

    p_result = None
    n = 0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=render_process_dir, shell=True)
        p_result = None
        while p_result is None:
            p_result = popen.poll()
            for stdout_line in iter(popen.stdout.readline, ""):
                if not len(stdout_line):
                    break
                line = stdout_line.decode('utf-8')
                if verbose:
                    print(line, end='')
                if logger_name != '':
                    logger.info(line)
                if raw_log_file != '':
                    log.write(line)

        n = n + 1
        popen.stdout.close()

    if len(raw_log_file):
        log.close()

    return p_result


def check_if_metered():
    cmd = ['ip', 'route']
    output = subprocess.check_output(cmd).decode(sys.stdout.encoding)

    output_lines = output.splitlines()
    if len(output_lines):
        return 'enx0' in output_lines[0]
    else:
        return False


def block_if_disabled():
    while os.path.exists(disable_flag):
        print('Waiting until disable_flag disappears')
        time.sleep(10)
    while not os.path.exists(local_status_txt_file):
        print('Warning: ' + local_status_txt_file + ' does not exist')
        time.sleep(10)


def system_was_monitoring_in_folder(folder):
    pats_xml_path = Path(folder, 'pats.xml')
    if not os.path.exists(pats_xml_path):
        print("Error: pats.xml not found in: " + folder)
        return False
    with open(pats_xml_path, "r", encoding="utf-8") as pats_xml:
        xml_lines = pats_xml.readlines()
        for line in xml_lines:
            if line.find('op_mode') != -1:
                if line.find('op_mode_c') != -1:
                    return True
                else:
                    print("Not a monitoring folder")
                    return False
    print("Error: op_mode not found in pats.xml not found in: " + folder)
    return False


def check_if_hunt_mode(pats_xml_path):
    with open(pats_xml_path, "r", encoding="utf-8") as pats_xml:
        xml_lines = pats_xml.readlines()
        for line in xml_lines:
            if line.find('op_mode_x') != -1:
                return True
    return False


def read_results_txt(results_txt_path):
    n_hunts = 0
    n_takeoffs = 0
    with open(results_txt_path, "r", encoding="utf-8") as results_txt:
        results_lines = results_txt.readlines()
        for line in results_lines:
            if line.find('n_hunts') != -1:
                n_hunts = int(line.split(':')[1])
            if line.find('n_takeoffs') != -1:
                n_takeoffs = int(line.split(':')[1])
    return n_hunts, n_takeoffs
