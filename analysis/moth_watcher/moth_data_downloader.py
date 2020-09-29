#!/usr/bin/env python3
import os,subprocess,time,datetime,glob,re
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

while True:
    download_jsons()
    print ('Downloaded jsons')
    cmd = ['~/code/pats/analysis/moth_watcher/fill_db_with_jsons.py -i ' + source_folder]
    result = subprocess.run(cmd, shell=True,stdout=subprocess.PIPE)
    print(result.returncode, result.stdout, result.stderr)
    download_renders_all()
    download_raw_logs_all()

    time.sleep(3600)
