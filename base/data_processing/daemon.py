#!/usr/bin/env python3
import time, threading,pause,os,time,logging,logging.handlers,abc,subprocess,sys,socket, re
from datetime import date, datetime, timedelta
from datetime import time as dttime
from pathlib import Path
import lib_base as lb
from clean_hd import clean_hd
from status_cc import send_status_update
from render_videos import render_last_day
from logs_to_json import process_all_logs_to_jsons,send_all_jsons
from cut_moths import cut_moths_all

status_cc_status_str = ''
def status_cc_worker():
    global status_cc_status_str
    logger = logging.getLogger('status_cc')
    logger.info('Status command center sender reporting in!')
    metered = lb.check_if_metered()
    logger.info('Metered mode: ' + str (metered))

    status_file_missing = not os.path.exists(lb.local_status_txt_file)
    if status_file_missing:
        logger.info('Warning: status file missing...')

    disabled_flag_detected = os.path.exists(lb.disable_flag)
    if disabled_flag_detected:
        logger.info('Warning: disable flag active...')

    #force an update at startup, otherwise it takes very long to pop up in the cc:
    if not disabled_flag_detected and not status_file_missing and lb.check_if_metered():
        send_status_update()

    while True:
        if status_file_missing == os.path.exists(lb.local_status_txt_file):
            status_file_missing = not os.path.exists(lb.local_status_txt_file)
            logger.info('Warning: status file went missing...')
        if disabled_flag_detected != os.path.exists(lb.disable_flag):
            disabled_flag_detected = os.path.exists(lb.disable_flag)
            logger.info('Warning: disable flag went active...')

        if not status_file_missing and not disabled_flag_detected:
            if lb.check_if_metered() != metered:
                logger.info('Metered mode now: ' + str (metered))
                metered = lb.check_if_metered()
            if (not metered):
                logger.info("Wifi detected. Updating now: " + datetime.today().strftime("%d-%m-%Y %H:%M:%S"))
                send_status_update()
                status_cc_status_str = 'Auto sent at ' + datetime.today().strftime("%d-%m-%Y %H:%M:%S")
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
    name=''
    periodic_td = timedelta()
    start_td=timedelta()
    status_str = ''
    thr = threading.Thread
    logger = logging.Logger
    error_cnt = 0


    def __init__(self,name, start_td, td,run_at_init):
        self.name = name
        self.periodic_td = td
        self.start_td=start_td
        self.logger = logging.getLogger(self.name)
        self.run_at_init = run_at_init

        self.thr = threading.Thread(target=self.do_work, daemon=True)
        self.thr.start()

    def next_trigger(self):
        if self.start_td.total_seconds() > 0:
            start_trigger_time = datetime.combine(date.today(), datetime.min.time())
            start_trigger_time = start_trigger_time + self.start_td
        else:
            start_trigger_time = datetime.combine(date.today(), datetime.min.time()) - self.periodic_td

        while start_trigger_time <= datetime.today():
            start_trigger_time = start_trigger_time + self.periodic_td

        return start_trigger_time

    def do_work(self):
        file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + self.name + '.log', maxBytes=1024*1024*100, backupCount=1)
        fh_errs = logging.handlers.TimedRotatingFileHandler(filename=lb.daily_errs_log, when='MIDNIGHT', backupCount=10, atTime=dttime(hour=10,minute=40))
        fh.setFormatter(file_format)
        fh.level = logging.DEBUG
        fh_errs.setFormatter(file_format)
        fh_errs.level = logging.ERROR
        fh_errs.suffix = "%Y%m%d" # Use the date as suffixs for old logs. e.g. all_errors.log.20210319.
        fh_errs.extMatch = re.compile(r"^\d{8}$") # Reformats the suffix such that it is predictable.
        self.logger.addHandler(fh)
        self.logger.addHandler(fh_errs)
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
            except Exception as e:
                self.logger.error(str(e))
                self.status_str = 'ERROR: ' + str(e)
                self.error_cnt += 1

            start_trigger_time = self.next_trigger()
            self.status_str = 'Ran at: ' + datetime.today().strftime("%d-%m-%Y %H:%M:%S") + ', next: ' + start_trigger_time.strftime("%d-%m-%Y %H:%M:%S")

    @abc.abstractmethod
    def task_func(self):
        pass

class clean_hd_task(pats_task):
    def __init__(self):
        super(clean_hd_task,self).__init__('clean_hd',timedelta(hours=8),timedelta(hours=24),False)

    def task_func(self):
        clean_hd()

class cut_moths_task(pats_task):
    def __init__(self):
        super(cut_moths_task,self).__init__('cut_moths',timedelta(),timedelta(minutes=60),False)

    def task_func(self):
        cut_moths_all()

class logs_to_json_task(pats_task):
    def __init__(self):
        super(logs_to_json_task,self).__init__('logs_to_json',timedelta(hours=10),timedelta(hours=24),False)

    def task_func(self):
        process_all_logs_to_jsons()
        send_all_jsons()

class errors_to_vps_task(pats_task):
    def __init__(self):
        super(errors_to_vps_task,self).__init__('errors_to_vps',timedelta(hours=10,minutes=45),timedelta(hours=24),False)

    def task_func(self):
        self.logger.error('Rotate!') #this forces the rotation of all_errors.log
        time.sleep(1)
        if not os.path.exists(lb.log_dir):
            os.mkdir(lb.log_dir)
        yesterday_file = lb.daily_errs_log + '.' + (datetime.today()-timedelta(days=1)).strftime("%Y%m%d")
        if os.path.exists(yesterday_file):
            remote_err_file='daily_basestation_errors/' + socket.gethostname() + '_' + lb.datetime_to_str(datetime.today()) + '.log'
            cmd = 'rsync -az ' + yesterday_file +' dash:' + remote_err_file
            lb.execute(cmd,5,logger_name=self.name)
            self.logger.info(yesterday_file + ' send to dash.')
class render_task(pats_task):
    def __init__(self):
        super(render_task,self).__init__('render',timedelta(hours=11),timedelta(hours=24),False)

    def task_func(self):
        render_last_day()

class wdt_pats_task(pats_task):
    def __init__(self):
        super(wdt_pats_task,self).__init__('wdt_pats',timedelta(),timedelta(seconds=300),False)
        self.no_realsense_cnt = 0

    def task_func(self):

        if not os.path.exists(lb.no_realsense_flag):
            self.no_realsense_cnt = 0
        else:
            os.remove(lb.no_realsense_flag)
            self.no_realsense_cnt += 1
            self.logger.warning('Could not find realsense. #' + str(self.no_realsense_cnt))
        if self.no_realsense_cnt >12: # 5 x 12 = 1 hour
                self.logger.error('Could not find realsense for over an hour! Rebooting...')
                cmd = 'sudo rtcwake -m off -s 120'
                lb.execute(cmd,1,logger_name=self.name)

        if os.path.exists(lb.proces_wdt_flag):
            os.remove(lb.proces_wdt_flag)
        else:
            self.error_cnt +=1
            self.logger.error('pats process watchdog alert! Pats Process does not seem to function. Restarting...')
            cmd = 'killall -9 pats'
            lb.execute(cmd,1,logger_name=self.name)

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

    def __init__(self):
        super(wdt_tunnel_task,self).__init__('wdt_tunnel',timedelta(),timedelta(hours=1),False)

    tunnel_ok_time = datetime.today()
    def task_func(self):
        cmd = 'lsof -i tcp:22'
        output = ''
        try:
            output = subprocess.check_output(cmd, shell=True)
        except:
            pass
        if output:
            output = output.decode(sys.stdout.encoding)
            output = output.splitlines()
        tunnel_ok = False

        for line in output:
            if line:
                if '->dash.pats-drones.com:ssh (ESTABLISHED)' in line:
                    self.tunnel_ok_time = datetime.today()
                    tunnel_ok = True

        if (datetime.today() - self.tunnel_ok_time).total_seconds() > 3*60*60 and datetime.today().hour == 13:
            self.error_cnt +=1
            self.logger.error('Tunnel watchdog alert! Rebooting!')
            cmd = 'sudo rtcwake -m off -s 120'
            lb.execute(cmd,1,logger_name=self.name)
        elif not tunnel_ok:
            self.error_cnt +=1
            self.logger.error('Tunnel watchdog alert! Holding of reboot until 13:00 though')


class check_system_task(pats_task):
    def __init__(self):
        super(check_system_task,self).__init__('check_system_task',timedelta(),timedelta(minutes=5),True)

    def task_func(self):
        cmd = 'sensors'
        output = subprocess.check_output(cmd, shell=True).decode(sys.stdout.encoding)
        output_lines = output.splitlines()
        for line in output_lines:
            if 'Package id 0' in line:
                cpu_temp_str = line.split(':')[1].split('(')[0].strip()
                temp = float(cpu_temp_str.replace('°C','').replace('+',''))
                if temp > 80:
                    self.logger.error('CPU Temperature too high: ' + cpu_temp_str)
                else:
                    self.logger.info('CPU Temperature: ' + cpu_temp_str)

def init_status_cc():
    file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    logger_status_cc = logging.getLogger('status_cc')
    fh = logging.handlers.RotatingFileHandler(filename=lb.log_dir + logger_status_cc.name + '.log', maxBytes=1024*1024*100, backupCount=1)
    fh.setFormatter(file_format)
    logger_status_cc.addHandler(fh)
    logger_status_cc.setLevel(logging.DEBUG)
    status_cc_thr = threading.Thread(target=status_cc_worker, daemon=True)
    status_cc_thr.start()


logging.basicConfig( )
lb.block_if_disabled()
if not os.path.exists(lb.log_dir):
    os.mkdir(lb.log_dir)
if not os.path.exists(lb.flags_dir):
    os.mkdir(lb.flags_dir)
init_status_cc()

tasks = []

tasks.append(clean_hd_task())
tasks.append(cut_moths_task())
tasks.append(logs_to_json_task())
tasks.append(render_task())
tasks.append(wdt_pats_task())
tasks.append(wdt_tunnel_task())
tasks.append(errors_to_vps_task())
tasks.append(check_system_task())

while True:
    os.system('clear')
    print('Status sender: ' + status_cc_status_str)

    for task in tasks:
        print(task.name + ': ' + task.status_str + ' #Errors:' + str(task.error_cnt))

    Path(lb.daemon_wdt_flag).touch()
    time.sleep(1)
