#!/usr/bin/env python3
import os,socket,subprocess,time,sys
from datetime import datetime, timedelta

homedir = os.environ['HOME']
hostname = socket.gethostname()

#folder, file and trigger string defs:
local_status_txt_file=homedir + '/pats_status.txt'
local_system_txt_file=homedir + '/pats_system_info.txt'
local_status_im_file=homedir + '/pats_monitor_tmp.jpg'
local_pats_xml=homedir + '/code/pats/xml/'
remote_status_txt_file='status/' + hostname + '/status.txt'
remote_system_txt_file='status/' + hostname + '/system.txt'
remote_status_im_file='status/' + hostname + '/status.jpg'
remote_pats_xml='status/' + hostname + '/'
pats_cc_update_request = homedir + '/pats_cc_update_request'
disable_pats_bkg = homedir + '/disable_pats_bkg'

def wait_for_trigger_or_timeout(delay):
    print("Periodic update after: " + str(delay))
    while not os.path.exists(pats_cc_update_request) and delay > 0 :
        time.sleep(0.2)
        delay = delay - 0.2
    if os.path.exists(pats_cc_update_request):
        print("Manual update trigger detected...")
        os.remove(pats_cc_update_request)

def get_ip():
    cmd = 'ip  -o -4 -f inet a show up primary scope global'
    output = subprocess.check_output(cmd, shell=True).decode(sys.stdout.encoding)
    ip = ''
    try:
        ip = output.split()[3].split('/')[0]
    except:
        pass
    return ip

first_read = ''
def send_status_update():
    global first_read
    f=open(local_status_txt_file, 'r')
    try:
        lines =f.readlines()
        if lines:
            if (lines[0] != first_read):
                first_read = lines[0]

                if os.path.exists(local_pats_xml):
                    cmd = 'rsync -az ' + local_pats_xml + ' mavlab-gpu:' + remote_pats_xml
                    subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
                if os.path.exists(local_status_txt_file):
                    cmd = 'rsync -z ' + local_status_txt_file +' mavlab-gpu:' + remote_status_txt_file
                    subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
                if os.path.exists(local_system_txt_file):
                    cmd = 'rsync -z ' + local_system_txt_file +' mavlab-gpu:' + remote_system_txt_file
                    subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
                if os.path.exists(local_status_im_file):
                    cmd = 'rsync -z ' + local_status_im_file +' mavlab-gpu:' + remote_status_im_file
                    subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
    except:
        pass

    f.close()

def update_monitor_results():
    now = datetime.now()
    yesterday = now - timedelta(days=1)
    date_time_start = yesterday.strftime("%Y%m%d_%H%M%S")
    date_time_end = now.strftime("%Y%m%d_%H%M%S")
    if not os.path.exists(homedir + '/data_json/'):
        os.mkdir(homedir + '/data_json/')
    local_json_file = homedir + '/data_json/' + date_time_end + '.json'
    remote_json_file='moth_json/' + hostname + '_' + date_time_end + '.json'
    cmd = '../analysis/moth_watcher/moth_counter.py -i ~/data -s ' + date_time_start +' -e ' + date_time_end + ' --filename ' + local_json_file
    result = subprocess.run(cmd, shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    print(result.returncode, result.stdout, result.stderr)
    cmd = 'rsync -z ' + local_json_file +' mavlab-gpu:' + remote_json_file
    subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)

while not os.path.exists(local_status_txt_file):
    time.sleep(10)
    print('Warning: ' + local_status_txt_file + ' does not exist')

updated_today = False
while True:

    while os.path.exists(disable_pats_bkg):
        time.sleep(10)
        print( 'Waiting until disable_pats_bkg disappears')

    if os.path.exists(local_status_txt_file): #f this file does not exist, the system is probably not functional
        send_status_update()

        now = datetime.now()
        if now.hour == 10 and not updated_today:
            updated_today = True
            update_monitor_results()
        if now.hour == 11 and updated_today:
            updated_today = False

    ip = get_ip()
    if ip.startswith('192.168.8'): #4g stick ip
        wait_for_trigger_or_timeout(3600)
    elif ip.startswith('192') or ip.startswith('172'):
        wait_for_trigger_or_timeout(1)
    else:
        wait_for_trigger_or_timeout(3600)

