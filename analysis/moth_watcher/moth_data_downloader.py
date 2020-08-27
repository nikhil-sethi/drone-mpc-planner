#!/usr/bin/python3
import os,subprocess,time

source_folder = '~/Downloads/moth_jsons'

def download(wait=False):
    global source_folder
    rsync_src='mavlab-gpu:/home/pats/moth_json/'
    subprocess.call(['mkdir -p ' + source_folder ], shell=True)
    cmd = ['rsync -zva --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ source_folder]
    if wait:
        subprocess.call(cmd, shell=True,stdout=subprocess.PIPE)
    else:
        subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)


while True:
    download()
    print ('Downloaded')
    cmd = ['~/code/pats/analysis/moth_watcher/moth_server.py -i ' + source_folder]
    result = subprocess.run(cmd, shell=True,stdout=subprocess.PIPE)
    print(result.returncode, result.stdout, result.stderr)
    time.sleep(3600)