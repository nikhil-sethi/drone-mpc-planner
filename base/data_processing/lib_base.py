import logging
import subprocess
import re
import sys
import os
import time
import socket
from datetime import datetime

flags_dir = os.path.expanduser('~/pats/flags/')
log_dir = os.path.expanduser('~/pats/logs/')
data_dir = os.path.expanduser('~/pats/data/')
json_dir = os.path.expanduser('~/pats/jsons/')
renders_dir = os.path.expanduser('~/pats/renders/')
pats_executable_dir = os.path.expanduser('~/code/pats/base/build/logging/')

daily_errs_log = log_dir + 'all_errors.log'
term_log_path = log_dir + 'term.log'
disable_flag = flags_dir + 'disable'
cc_update_request = flags_dir + 'cc_update_request'
daemon_wdt_flag = flags_dir + 'daemon_wdt_flag'
proces_wdt_flag = flags_dir + 'proces_wdt_flag'
no_realsense_flag = flags_dir + 'no_realsense_flag'
reset_realsense_flag = flags_dir + 'reset_realsense_flag'
baseboard_updated_flag = flags_dir + 'baseboard_software_uploaded'

wdt_fired_flag = pats_executable_dir + 'wdt_fired'

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


def datetime_to_str(d):
    return d.strftime('%Y%m%d_%H%M%S')


def natural_sort(line):
    def convert(text):
        return int(text) if text.isdigit() else text.lower()

    def alphanum_key(key):
        return [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(line, key=alphanum_key)


def execute(cmd, retry=1, logger_name='', render_process_dir=None, verbose=True):
    if logger_name != '':
        logger = logging.getLogger(logger_name)

    p_result = None
    n = 0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, cwd=render_process_dir, shell=True)
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result is not None:
                n = n + 1
                break
            if verbose and logger_name == '':
                print(stdout_line.decode('utf-8'), end='')
            elif logger_name != '':
                logger.info(stdout_line.decode('utf-8'))
        popen.stdout.close()
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
