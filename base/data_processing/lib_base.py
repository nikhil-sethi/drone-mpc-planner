import logging,subprocess,re,sys,os,time,socket
from datetime import datetime

flags_dir = os.path.expanduser('~/pats/flags/')
log_dir = os.path.expanduser('~/pats/logs/')
data_dir =  os.path.expanduser('~/pats/data/')
json_dir = os.path.expanduser('~/pats/jsons/')
renders_dir = os.path.expanduser('~/pats/renders/')

daily_errs_log = log_dir + 'all_errors.log'
term_log_path =  log_dir + 'term.log'
disable_flag = flags_dir + 'disable'
cc_update_request = flags_dir + 'cc_update_request'
daemon_wdt_flag =flags_dir + 'daemon_wdt_flag'
proces_wdt_flag =flags_dir + 'proces_wdt_flag'
no_realsense_flag =flags_dir + 'no_realsense_flag'

local_status_txt_file=os.path.expanduser('~/pats/status/status.txt')
local_system_txt_file=os.path.expanduser('~/pats/status/system_info.txt')
local_status_im_file=os.path.expanduser('~/pats/status/monitor_tmp.jpg')
local_pats_xml=os.path.expanduser('~/code/pats/base/xml/')
remote_status_txt_file='status/' + socket.gethostname() + '/status.txt'
remote_system_txt_file='status/' + socket.gethostname() + '/system.txt'
remote_status_im_file='status/' + socket.gethostname() + '/status.jpg'
remote_pats_xml='status/' + socket.gethostname() + '/'


def str_to_datetime(string):
    return datetime.strptime(string, "%Y%m%d_%H%M%S")
def datetime_to_str(d):
    return d.strftime('%Y%m%d_%H%M%S')

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)

def execute(cmd,retry=1,logger_name=''):
    if logger_name != '':
        logger = logging.getLogger(logger_name)

    p_result = None
    n=0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result != None:
                n = n+1
                break
            if logger_name == '':
                print(stdout_line.decode('utf-8'),end ='')
            else:
                logger.info(stdout_line.decode('utf-8'))
        popen.stdout.close()
    return p_result

def check_if_metered():
    cmd = 'ip route'
    output = subprocess.check_output(cmd, shell=True).decode(sys.stdout.encoding)
    ip = ''

    output_lines = output.splitlines()
    if len(output_lines):
        return 'enx0' in output_lines[0]
    else:
        return False

def block_if_disabled():
    while os.path.exists(disable_flag):
        print( 'Waiting until disable_flag disappears')
        time.sleep(10)
    while not os.path.exists(local_status_txt_file):
        print('Warning: ' + local_status_txt_file + ' does not exist')
        time.sleep(10)
