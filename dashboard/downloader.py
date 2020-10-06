#!/usr/bin/env python3
import os,subprocess,time,datetime,glob,re,argparse
from tqdm import tqdm
source_folder = '~/Downloads/moth_jsons'

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

def download_jsons():
    global source_folder
    rsync_src='mavlab-gpu:/home/pats/moth_json/'
    subprocess.call(['mkdir -p ' + source_folder ], shell=True)
    cmd = ['rsync -zvaP --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ source_folder]
    execute(cmd)

def download_renders(pats_id):
    cmd = ['rsync -azvP pats' + str(pats_id) + ':data_rendered/* ~/Downloads/pats_renders']
    execute(cmd)
def download_renders_all():
    download_renders(11)
    download_renders(12)
    download_renders(13)
    download_renders(14)
    download_renders(15)
    download_renders(16)
    download_renders(17)
    download_renders(18)
    download_renders(19)
    download_renders(21)

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def download_raw_logs_all():
    target_folder = '~/Downloads/pats_raw_logs/'
    subprocess.call(['mkdir -p ' + target_folder ], shell=True)
    files = natural_sort([fp for fp in glob.glob(os.path.expanduser('~/Downloads/pats_renders/*.mp4'))])
    pbar= tqdm(files)
    for file_ in pbar:
        foldername = os.path.basename(file_)[:15]
        systemname = os.path.basename(file_).split('_')[2].split('.')[0].replace('-proto','')
        if not os.path.exists(os.path.expanduser(target_folder + foldername + '/')):
            cmd = ['rsync -zvaP ' + systemname +':data/' + foldername + ' ' + target_folder]
            execute(cmd,5)


parser = argparse.ArgumentParser(description='Script that periodically downloads data from deployed systems')
parser.add_argument('-j', '--download_jsons', action='store_true')
parser.add_argument('-r', '--download_renders', action='store_true')
parser.add_argument('-l', '--download_raw_logs', action='store_true')
args = parser.parse_args()

while True:
    
    if args.download_jsons:
        download_jsons()
        print ('Downloaded jsons')
    
    cmd = ['~/code/pats/dashboard/jsons_to_db.py -i ' + source_folder]
    execute(cmd)

    if args.download_renders:
        download_renders_all()
        if args.download_raw_logs:
            download_raw_logs_all()

    time.sleep(3600)
