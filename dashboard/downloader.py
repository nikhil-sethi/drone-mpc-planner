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
    rsync_src='dash:/home/pats/jsons/'
    subprocess.call(['mkdir -p ' + source_folder ], shell=True)
    cmd = ['rsync -zvaP --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ source_folder]
    execute(cmd)

def download_db():
    global source_folder
    rsync_src='dash:/home/pats/pats.db'
    cmd = ['rsync -zvaP --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ '~/pats.db']
    execute(cmd)

def download_renders(pats_id):
    cmd = ['rsync -azvP pats' + str(pats_id) + ':data_rendered/* ~/Downloads/pats_renders']
    execute(cmd)
def download_renders_all():
    download_renders(30)
    download_renders(31)
    download_renders(32)
    download_renders(33)
    download_renders(34)
    download_renders(35)
    download_renders(36)
    download_renders(37)
    download_renders(38)
    download_renders(39)

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def download_raw_logs_all():
    target_folder = '~/Downloads/pats_raw_logs/'
    subprocess.call(['mkdir -p ' + target_folder ], shell=True)
    files = natural_sort([fp for fp in glob.glob(os.path.expanduser('~/Downloads/pats_renders/*.mp4'))])
    files += natural_sort([fp for fp in glob.glob(os.path.expanduser('~/Downloads/pats_renders/*.mkv'))])
    pbar= tqdm(files)
    for file_ in pbar:
        foldername = os.path.basename(file_)[:15]
        systemname = os.path.basename(file_).split('_')[2].split('.')[0].replace('-proto','')
        if not os.path.exists(os.path.expanduser(target_folder + foldername + '/')):
            cmd = ['rsync -zvaP ' + systemname +':data/' + foldername + ' ' + target_folder]
            execute(cmd,5)


parser = argparse.ArgumentParser(description='Script that periodically downloads data from deployed systems')

args = parser.parse_args()

while True:

    download_jsons()
    print ('Downloaded jsons')
    download_db()
    print ('Downloaded db')
    download_renders_all()
    print ('Downloaded renders')
    download_raw_logs_all()
    print ('Downloaded raw logs')

    time.sleep(3600)
