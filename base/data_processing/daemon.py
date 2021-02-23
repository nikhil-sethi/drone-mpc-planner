#!/usr/bin/env python3
import time, threading,pause,os,time,logging,logging.handlers,abc
from datetime import date, datetime, timedelta
from pathlib import Path
import lib_base as lb
from clean_hd import clean_hd
from status_cc import send_status_update
from render_videos import render_last_day
from logs_to_json import send_json_of_last_day
from cut_moths import cut_moths_current_night

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

    def __init__(self,name, start_td, td):
        self.name = name
        self.periodic_td = td
        self.start_td=start_td
        self.logger = logging.getLogger(self.name)

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
        fh.setFormatter(file_format)
        self.logger.addHandler(fh)
        self.logger.setLevel(logging.DEBUG)
        self.logger.info(self.name + ' reporting in!')
        start_trigger_time = self.next_trigger()
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
        super(clean_hd_task,self).__init__('clean_hd',timedelta(hours=8),timedelta(hours=24))

    def task_func(self):
        clean_hd()

class cut_moths_task(pats_task):

    def __init__(self):
        super(cut_moths_task,self).__init__('cut_moths',timedelta(),timedelta(minutes=15))

    def task_func(self):
        cut_moths_current_night()

class logs_to_json_task(pats_task):

    def __init__(self):
        super(logs_to_json_task,self).__init__('logs_to_json',timedelta(hours=10),timedelta(hours=24))

    def task_func(self):
        send_json_of_last_day()

class render_task(pats_task):

    def __init__(self):
        super(render_task,self).__init__('render',timedelta(hours=11),timedelta(hours=24))

    def task_func(self):
        render_last_day()

class wdt_task(pats_task):

    def __init__(self):
        super(wdt_task,self).__init__('wdt',timedelta(),timedelta(seconds=300))

    def task_func(self):
        if os.path.exists(lb.proces_wdt_flag):
            os.remove(lb.proces_wdt_flag)
        else:
            self.error_cnt +=1
            self.logger.error('pats process watchdog alert! Pats Process does not seem to function. Restarting...')
            cmd = 'killall -9 pats'
            lb.execute(cmd,1,logger_name=self.name)

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
init_status_cc()
clean_hd_worker = clean_hd_task()
cut_moth_worker = cut_moths_task()
logs_to_json_worker = logs_to_json_task()
render_worker = render_task()
wdt_worker = wdt_task()

while True:
    os.system('clear')
    print('Status sender: ' + status_cc_status_str)
    print('HD cleaner: ' + clean_hd_worker.status_str + ' #Errors:' + str(clean_hd_worker.error_cnt))
    print('Moth cutter: ' + cut_moth_worker.status_str + ' #Errors:' + str(cut_moth_worker.error_cnt))
    print('Json processor: ' + logs_to_json_worker.status_str + ' #Errors:' + str(logs_to_json_worker.error_cnt))
    print('Renderer: ' + render_worker.status_str + ' #Errors:' + str(render_worker.error_cnt))
    print('WDT: ' + wdt_worker.status_str + ' #Errors:' + str(wdt_worker.error_cnt))

    Path(lb.daemon_wdt_flag).touch()
    time.sleep(1)
