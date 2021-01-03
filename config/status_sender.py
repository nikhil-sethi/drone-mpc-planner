#!/usr/bin/env python3
import os,socket,subprocess,time,sys,shutil,glob,re,argparse
from datetime import datetime, timedelta
from pathlib import Path

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

def execute(cmd,retry=1):
    p_result = None
    n=0
    while p_result != 0 and n < retry:
        popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
        for stdout_line in iter(popen.stdout.readline, ""):
            p_result = popen.poll()
            if p_result != None:
                n = n+1
                break
            print(stdout_line.decode('utf-8'),end ='')
        popen.stdout.close()

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
                    cmd = 'rsync -puaz ' + local_pats_xml + ' dash:' + remote_pats_xml
                    execute(cmd)
                if os.path.exists(local_status_txt_file):
                    cmd = 'rsync -pz ' + local_status_txt_file +' dash:' + remote_status_txt_file
                    execute(cmd)
                if os.path.exists(local_system_txt_file):
                    cmd = 'rsync -pz ' + local_system_txt_file +' dash:' + remote_system_txt_file
                    execute(cmd)
                if os.path.exists(local_status_im_file):
                    cmd = 'rsync -puz ' + local_status_im_file +' dash:' + remote_status_im_file
                    execute(cmd)
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
    remote_json_file='jsons/' + hostname + '_' + date_time_end + '.json'
    cmd = '../config/log_to_json.py -i ~/data -s ' + date_time_start +' -e ' + date_time_end + ' --filename ' + local_json_file
    execute(cmd)
    cmd = 'rsync -puz ' + local_json_file +' dash:' + remote_json_file
    execute(cmd,5)

def render_hunts():
    now = datetime.now()
    yesterday = now - timedelta(days=1)
    date_time_start = yesterday.strftime("%Y%m%d_%H%M%S")
    date_time_end = now.strftime("%Y%m%d_%H%M%S")
    cmd = '../config/render_videos.py -i ~/data -s ' + date_time_start +' -e ' + date_time_end
    execute(cmd)

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)

def clean_hd():
    data_dir =  os.path.expanduser('~/data/')
    deamon_log =  os.path.expanduser('~/pats_daemon.log')
    while True:
        total_used_space,_,free_space = shutil.disk_usage(data_dir)
        print('Free space: ' + str(free_space / 1024 / 1024 / 1024) + 'GB --> ' +str(round(free_space /  total_used_space * 100)) + '% free')
        if (free_space /  total_used_space < 0.2):
            if os.path.exists(deamon_log):
                if Path(deamon_log).stat().st_size > 1024*1024*1024: #1GB
                    os.remove(deamon_log)
            found_dirs = natural_sort(glob.glob(data_dir + "/202*_*"))
            for dir in found_dirs:
                try:
                    dir_date =  datetime.strptime(os.path.basename(dir),"%Y%m%d_%H%M%S")
                except :
                    shutil.rmtree(dir)
                    break
                if ((datetime.now() - dir_date) > timedelta(days=14)):
                    print('removing: ' + dir)
                    if os.path.exists(dir + '/logging'):
                        shutil.rmtree(dir)
                        break
                    shutil.rmtree(dir)
                else:
                    return
        else:
            return

def check_if_metered():
    cmd = 'ip route'
    output = subprocess.check_output(cmd, shell=True).decode(sys.stdout.encoding)
    ip = ''

    output_lines = output.splitlines()
    return 'enx0c5b8f279a64' in output_lines[0]

def block_if_disabled():
    while os.path.exists(disable_pats_bkg):
        time.sleep(10)
        print( 'Waiting until disable_pats_bkg disappears')

parser = argparse.ArgumentParser(description='Headless status sender to dash and commandcenter')
parser.add_argument('-t','--hour', help="Update results at the start of this hour.", default=9)
args = parser.parse_args()

while not os.path.exists(local_status_txt_file):
    time.sleep(10)
    print('Warning: ' + local_status_txt_file + ' does not exist')

updated_today = False
metered_connection = check_if_metered()
if metered_connection:
   print("Metered connection mode. Updating at: " + str(args.hour) + ':00')
print('Starting status sender with render update at ' + str(args.hour))
print('\n')
while True:

    block_if_disabled()
    metered_connection = check_if_metered()

    if os.path.exists(local_status_txt_file): #if this file does not exist, the system is probably not functional

        now = datetime.now()
        if (not metered_connection):
            print ("\033[A                             \033[A")
            print("Updating now: " + str(now))
            send_status_update()
        else:
            print ("\033[A                                                        \033[A")
            print("Metered connection mode. Next update at: " + str(args.hour))

        if os.path.exists(pats_cc_update_request):
            print("\nManual update trigger detected...\n")
            os.remove(pats_cc_update_request)
            send_status_update()

        if (now.hour == int(args.hour) and not updated_today):
            if (metered_connection):
                print("\nMetered connection mode. Updating now: " + str(now))
                send_status_update()
            print("\nStarting data aggregation: " + str(now) + '\n')
            update_monitor_results()
            render_hunts()
            clean_hd()
            updated_today = True

        if now.hour != int(args.hour) and updated_today:
            print("\nResetting updated_today flag: " + str(now) + '\n')
            updated_today = False

    time.sleep(1)
