#!/usr/bin/env python3
import os
import time
import threading
from typing import List
import logging
import logging.handlers
import abc
import subprocess
import sys
import struct
import socket
import re
import shutil
from datetime import date, datetime, timedelta
from pathlib import Path
import pandas as pd
import pause
from lib_socket import socket_communication
import lib_base as lb
from clean_hd import clean_hd
from status_cc import send_status_update
from render_videos import render_last_day
from aggregate_jsons import aggregate_jsons, send_all_jsons
import lib_serialization as ls
sys.path.append('ai')  # noqa

status_cc_status_str = '-'
daemon2baseboard_pkg = ls.SocketDaemon2BaseboardLinkPackage()
executor2daemon_pkg = ls.SocketExecutorStatePackage()


def status_cc_worker():
    global status_cc_status_str  # pylint: disable=global-statement
    logger = logging.getLogger('status_cc')
    logger.info('Status command center sender reporting in!')

    status_file_missing = not os.path.exists(lb.local_status_txt_file)
    if status_file_missing:
        logger.info('Warning: status file missing...')

    disabled_flag_detected = os.path.exists(lb.disable_flag)
    if disabled_flag_detected:
        logger.info('Warning: disable flag active: ' + lb.disable_flag)

    # force an update at startup, otherwise it takes very long to pop up in the cc:
    if not disabled_flag_detected and not status_file_missing:
        send_status_update()

    while True:
        if status_file_missing == os.path.exists(lb.local_status_txt_file):
            status_file_missing = not os.path.exists(lb.local_status_txt_file)
            logger.info('Warning: status file went missing...')
        if disabled_flag_detected != os.path.exists(lb.disable_flag):
            disabled_flag_detected = os.path.exists(lb.disable_flag)
            logger.info('Warning: disable flag went active...')

        if not status_file_missing and not disabled_flag_detected:

            if os.path.exists(lb.cc_update_request):
                logger.info("Manual update trigger detected at " + datetime.today().strftime("%d-%m-%Y %H:%M:%S"))
                os.remove(lb.cc_update_request)
                send_status_update()
                status_cc_status_str = 'Triggered at' + datetime.today().strftime("%d-%m-%Y %H:%M:%S")
        elif disabled_flag_detected:
            status_cc_status_str = 'Disabled'
        else:
            status_cc_status_str = 'Status file missing!'
        time.sleep(1)


class pats_task(metaclass=abc.ABCMeta):
    name = ''
    periodic_td = timedelta()
    start_td = timedelta()
    status_str = ''
    thr = threading.Thread
    logger = logging.Logger
    error_cnt = 0

    def __init__(self, name, start_td, td, run_at_init, error_file_handler):
        self.name = name
        self.periodic_td = td
        self.start_td = start_td
        self.logger = logging.getLogger(self.name)
        self.run_at_init = run_at_init
        self.error_file_handler = error_file_handler

        self.thr = threading.Thread(target=self.do_work, daemon=True)
        self.thr.start()

    def next_trigger(self):
        if self.start_td.total_seconds() > 0:
            start_trigger_time = datetime.combine(date.today(), (datetime.min + self.start_td).time())
            while pd.to_datetime(start_trigger_time).round('1s') <= pd.to_datetime(datetime.today()).round('1s'):
                start_trigger_time = start_trigger_time + self.periodic_td
        else:
            start_trigger_time = datetime.combine(date.today(), datetime.today().time()) + self.periodic_td

        return start_trigger_time

    def do_work(self):
        file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + self.name + '.log', maxBytes=1024 * 1024 * 100, backupCount=1)
        fh.setFormatter(file_format)
        fh.level = logging.DEBUG
        self.logger.addHandler(fh)
        self.logger.addHandler(self.error_file_handler)
        self.logger.setLevel(logging.DEBUG)
        self.logger.info(self.name + ' reporting in!')
        start_trigger_time = self.next_trigger()
        if self.run_at_init:
            start_trigger_time = datetime.today()
        self.status_str = 'First start at: ' + start_trigger_time.strftime("%d-%m-%Y %H:%M:%S")

        while True:
            self.logger.info('Waiting until ' + start_trigger_time.strftime("%d-%m-%Y %H:%M:%S"))
            pause.until(start_trigger_time)
            try:
                self.task_func()
            except Exception as e:  # pylint: disable=broad-except
                self.logger.error(str(e))
                self.status_str = 'ERROR: ' + str(e)
                self.error_cnt += 1
            start_trigger_time = self.next_trigger()
            self.status_str = 'Ran at: ' + datetime.today().strftime("%d-%m-%Y %H:%M:%S") + ', next: ' + start_trigger_time.strftime("%d-%m-%Y %H:%M:%S")

    @abc.abstractmethod
    def task_func(self):
        pass


class clean_hd_task(pats_task):
    def __init__(self, error_file_handler):
        super(clean_hd_task, self).__init__('clean_hd', timedelta(hours=8), timedelta(hours=1), False, error_file_handler)

    def task_func(self):
        clean_hd()


class wp_demo_task(pats_task):
    def __init__(self, error_file_handler):
        super(wp_demo_task, self).__init__('wp_demo', timedelta(hours=1), timedelta(minutes=15), False, error_file_handler)

    def task_func(self):
        if os.path.exists(lb.enable_wp_demo_flag):
            shutil.copyfile(os.path.expanduser('~/code/pats/base/xml/flightplans/simple-demo-floriade.xml'), os.path.expanduser('~/pats/flags/pats_demo.xml'))


class baseboard_task(pats_task):
    def __init__(self, error_file_handler):
        super(baseboard_task, self).__init__('baseboard', timedelta(), timedelta(seconds=3), False, error_file_handler)

    def task_func(self):
        if not os.path.exists(lb.disable_baseboard_flag):
            baseboard_comm.send(daemon2baseboard_pkg.pack())


class aggregate_jsons_task(pats_task):
    def __init__(self, error_file_handler):
        super(aggregate_jsons_task, self).__init__('aggregate_jsons', timedelta(hours=8, minutes=30), timedelta(hours=24), False, error_file_handler)

    def task_func(self):
        daemon2baseboard_pkg.post_processing = 1
        aggregate_jsons(lb.data_dir, socket.gethostname(), lb.json_dir + lb.datetime_to_str_with_timezone(datetime.now()) + '.json')
        tries = 0
        retry = True
        while retry and tries < 5:
            if send_all_jsons() == 0:
                retry = False
            else:
                tries += 1
                self.logger.warning('Sending error file failed retrying in 10 min.')
                pause.until(datetime.today() + timedelta(minutes=10))
        if retry:
            self.logger.error('Sending jsons failed 5 times.')
        daemon2baseboard_pkg.post_processing = 0


class errors_to_vps_task(pats_task):
    def __init__(self, error_file_handler, rotate_time):
        super(errors_to_vps_task, self).__init__('errors_to_vps', timedelta(hours=rotate_time.hour, minutes=rotate_time.minute + 5), timedelta(hours=24), False, error_file_handler)

    def task_func(self):
        self.logger.error('Rotate!')  # this forces the rotation of all_errors.log
        time.sleep(1)
        if not os.path.exists(lb.log_dir):
            os.mkdir(lb.log_dir)
        yesterday_file = lb.daily_errs_log + '.' + (datetime.today() - timedelta(days=1)).strftime("%Y%m%d")
        if os.path.exists(yesterday_file):
            remote_err_file = 'daily_basestation_errors/' + socket.gethostname() + '_' + lb.datetime_to_str_with_timezone(datetime.today()) + '.log'
            cmd = 'rsync -az ' + yesterday_file + ' dash:' + remote_err_file
            tries = 0
            retry = True
            while retry and tries < 5:
                if lb.execute(cmd, 5, logger_name=self.name) == 0:
                    self.logger.info(yesterday_file + ' send to dash.')
                    retry = False
                else:
                    tries += 1
                    self.logger.warning('Sending error file failed retrying in 10 min.')
                    pause.until(datetime.today() + timedelta(minutes=10))
            if retry:
                self.logger.error('Sending error file failed 5 times.')
        else:
            self.logger.error('Error file rotation did not work')


class render_task(pats_task):
    def __init__(self, error_file_handler):
        super(render_task, self).__init__('render', timedelta(hours=9), timedelta(hours=24), False, error_file_handler)

    def task_func(self):
        if not os.path.exists(lb.disable_rendering_flag):
            daemon2baseboard_pkg.rendering = 1
            render_last_day(abort_deadline=datetime.now() + timedelta(hours=7))
            daemon2baseboard_pkg.rendering = 0
        else:
            self.logger.info('Rendering disabled.')


class wdt_pats_task(pats_task):

    def __init__(self, error_file_handler, baseboard_comm):
        self.baseboard_comm = baseboard_comm
        super(wdt_pats_task, self).__init__('wdt_pats', timedelta(), timedelta(seconds=300), False, error_file_handler)
        self.no_realsense_cnt = 0
        self.last_realsense_reset = datetime.min

    def task_func(self):
        if os.path.exists(lb.disable_flag):
            return

        dt_last_executor_msg = (datetime.now() - executor_comm.last_msg_time).total_seconds()
        if dt_last_executor_msg > 60:
            self.error_cnt += 1
            self.logger.error('executor process watchdog alert! Executor does not seem to function. Restarting...')
            Path(lb.executor_log_dir).mkdir(parents=True, exist_ok=True)
            cmd = 'killall -9 executor'
            lb.execute(cmd, 1, logger_name=self.name)
        elif executor2daemon_pkg.executor_state != ls.executor_states.es_realsense_error:
            self.no_realsense_cnt = 0
        else:
            self.no_realsense_cnt += 1
            self.logger.warning('Could not find realsense. #' + str(self.no_realsense_cnt))

        if self.no_realsense_cnt > 12:  # 5 x 12 = 1 hour
            self.logger.error('Could not find realsense for over an hour! Rebooting...')
            cmd = 'sudo rtcwake -m off -s 120'
            lb.execute(cmd, 1, logger_name=self.name)

        if executor2daemon_pkg.executor_state == ls.executor_states.es_realsense_reset:
            self.logger.info('Realsense was reset')
            self.last_realsense_reset = datetime.now()


class wdt_tunnel_task(pats_task):
    # What I want to happen is the following:
    # - restart the NUC if the tunnel is failing, but:
    # - only if absolutely necessary and it does not hurt the monitoring operation
    # - and make sure it can never constantly restart, so that we have a good chance
    #   of re establishing contact manually if there is some unforeseen problem
    #   with the wdt or whatever

    # On the other hand, I do want to know if there are problems with tunnel more
    # often. So, it is logged once per hour, but only once per day is it allowed to
    # actually reboot itself. This moment at 13:00 is chosen such that it can't
    # influence the monitoring results. The post processing scripts should be done by
    # then, and the system is just waiting for darkness.

    def __init__(self, error_file_handler):
        super(wdt_tunnel_task, self).__init__('wdt_tunnel', timedelta(), timedelta(hours=1), True, error_file_handler)

    tunnel_ok_time = datetime.today()

    def task_func(self):
        if os.path.exists(lb.disable_tunnel_flag):
            daemon2baseboard_pkg.internet_OK = 1
            return

        tunnel_ok = False
        cmd = 'lsof -i tcp:22'
        output = ''
        try:
            output = subprocess.check_output(cmd.split(' '))
        except Exception as e:  # pylint: disable=broad-except
            self.logger.warning('Error in getting tunnel info: ' + str(e))
        if output:
            output = output.decode(sys.stdout.encoding)
            output = output.splitlines()
            for line in output:
                if line:
                    if '->dash.pats-drones.com:ssh (ESTABLISHED)' in line:
                        self.tunnel_ok_time = datetime.today()
                        tunnel_ok = True

        ping_ok = False
        cmd = 'ping -c3 pats-c.com'
        output = ''
        try:
            output = subprocess.check_output(cmd.split(' '))
        except Exception as e:  # pylint: disable=broad-except
            self.logger.warning('Error in getting ping info: ' + str(e))
        if output:
            output = output.decode(sys.stdout.encoding)
            output = output.splitlines()
            for line in output:
                if line:
                    if 'packets transmitted, ' in line:
                        p = int(line.split(',')[2].strip().split('%')[0])
                        if p < 90:
                            ping_ok = True
                            self.logger.info(line)
                            break

        if tunnel_ok and ping_ok:
            daemon2baseboard_pkg.internet_OK = 1
        else:
            daemon2baseboard_pkg.internet_OK = 0

        if (datetime.today() - self.tunnel_ok_time).total_seconds() > 3 * 60 * 60 and datetime.today().hour == 13:
            self.error_cnt += 1
            self.logger.error('Tunnel watchdog alert! Rebooting!')
            cmd = 'sudo rtcwake -m off -s 120'
            lb.execute(cmd, 1, logger_name=self.name)
        elif not tunnel_ok:
            self.error_cnt += 1
            self.logger.warning('Tunnel watchdog alert! Holding of reboot until 13:00 though')


class check_system_task(pats_task):
    def __init__(self, error_file_handler):
        super(check_system_task, self).__init__('check_system_task', timedelta(), timedelta(minutes=5), True, error_file_handler)

    def task_func(self):
        cmd = 'sensors'
        output = subprocess.check_output(cmd).decode(sys.stdout.encoding)
        output_lines = output.splitlines()
        for line in output_lines:
            if 'Package id 0' in line:
                cpu_temp_str = line.split(':')[1].split('(')[0].strip()
                temp = float(cpu_temp_str.replace('°C', '').replace('+', ''))
                if temp > 80:
                    self.logger.error('CPU Temperature too high: ' + cpu_temp_str)
                else:
                    self.logger.info('CPU Temperature: ' + cpu_temp_str)

        _, _, free = shutil.disk_usage("/")
        if free / (2**30) < 10:
            self.logger.error('SSD almost full. Free space left: ' + str(free / (2**30)) + 'GB')


def init_status_cc():
    file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    logger_status_cc = logging.getLogger('status_cc')
    fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + logger_status_cc.name + '.log', maxBytes=1024 * 1024 * 100, backupCount=1)
    fh.setFormatter(file_format)
    logger_status_cc.addHandler(fh)
    logger_status_cc.setLevel(logging.DEBUG)
    status_cc_thr = threading.Thread(target=status_cc_worker, daemon=True)
    status_cc_thr.start()


def executor_receive(msg):
    while len(msg) > 3:
        if msg[0] == ord(ls.EXECUTOR_PACKAGE_PRE_HEADER):
            if msg[1] == ord(ls.executor_package_headers.header_SocketExecutorStatePackage.value[0]):
                executor2daemon_pkg.parse(msg[:struct.calcsize(executor2daemon_pkg.format)])
                msg = msg[struct.calcsize(executor2daemon_pkg.format):]
            else:
                msg = ''
                logger.warning('Weird package received from executor...')
        else:
            msg = msg[1:]
            logger.warning('Some weird byte was received from executor...')


logging.basicConfig()
if not os.path.exists(lb.log_dir):
    os.mkdir(lb.log_dir)
if not os.path.exists(lb.flags_dir):
    os.mkdir(lb.flags_dir)
init_status_cc()

rotate_time = datetime(1, 1, 1, hour=9, minute=25)  # for the rotate time only hour and minute are used so year, month and day are irrelevant
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
error_file_handler = logging.handlers.TimedRotatingFileHandler(filename=lb.daily_errs_log, when='MIDNIGHT', backupCount=10, atTime=rotate_time)
error_file_handler.setFormatter(file_format)
error_file_handler.level = logging.ERROR
error_file_handler.suffix = "%Y%m%d"  # Use the date as suffixs for old logs. e.g. all_errors.log.20210319.
error_file_handler.extMatch = re.compile(r"^\d{8}$")  # Reformats the suffix such that it is predictable.

logger = logging.getLogger('daemon')
fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + 'daemon.log', maxBytes=1024 * 1024 * 100, backupCount=1)
fh.setFormatter(file_format)
logger.addHandler(fh)
logger.addHandler(error_file_handler)
logger.setLevel(logging.DEBUG)
start_time = datetime.today().strftime("%d-%m-%Y %H:%M:%S")
logger.info('Daemon reporting in!')
print(start_time)
start_sha = subprocess.check_output(["git", "describe"]).decode(sys.stdout.encoding).strip()
logger.info('sha: ' + start_sha)

baseboard_comm = socket_communication('baseboard', 'daemon', lb.socket_baseboard2daemon, False)
executor_comm = socket_communication('executor', 'daemon', lb.socket_executor2daemon, True, executor_receive)

tasks: List[pats_task] = []
tasks.append(wdt_pats_task(error_file_handler, baseboard_comm))
lb.block_if_disabled()  # We need the wdt_pats to write to the baseboard, but the rest can wait until the disable flag is removed.
tasks.append(clean_hd_task(error_file_handler))
tasks.append(aggregate_jsons_task(error_file_handler))
tasks.append(render_task(error_file_handler))
tasks.append(wdt_tunnel_task(error_file_handler))
tasks.append(errors_to_vps_task(error_file_handler, rotate_time))
tasks.append(check_system_task(error_file_handler))
tasks.append(wp_demo_task(error_file_handler))
tasks.append(baseboard_task(error_file_handler))


while True:
    os.system('clear')  # nosec
    print(datetime.today().strftime("%d-%m-%Y %H:%M:%S"))
    print('PATS daemon ' + start_sha + '. Started: ' + start_time)
    print('CC status sender: ' + status_cc_status_str)
    if os.path.exists(lb.disable_baseboard_flag):
        print('Baseboard: disabled')
    else:
        if not baseboard_comm.connection_ok:
            print('Baseboardlink LOST since: ' + baseboard_comm.connection_lost_datetime.strftime("%d-%m-%Y %H:%M:%S"))
        else:
            print('Baseboard communication OK')

    if os.path.exists(lb.disable_daemonlink_flag):
        print('Executor daemon link: disabled by flag: ' + lb.disable_daemonlink_flag)
    elif not executor_comm.connection_ok:
        print('Executor daemon link LOST since: ' + executor_comm.connection_lost_datetime.strftime("%d-%m-%Y %H:%M:%S"))
    else:
        print('Executor daemon link OK: ' + str(round(executor2daemon_pkg.time)))

    for task in tasks:
        print(task.name + ': ' + task.status_str + ' #Errors:' + str(task.error_cnt))

    current_sha = subprocess.check_output(["git", "describe"]).decode(sys.stdout.encoding).strip()
    if start_sha != current_sha:
        logger.warning("SHA discrepancy detected!")

    time.sleep(1)
