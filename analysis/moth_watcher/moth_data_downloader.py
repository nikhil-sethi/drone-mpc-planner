#!/usr/bin/env python3
import os,subprocess,time

source_folder = '~/Downloads/moth_jsons'

def execute(cmd):
    popen = subprocess.Popen(cmd, shell=True,stderr=subprocess.STDOUT,stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        if popen.poll() != None:
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()
    return

def download_jsons():
    global source_folder
    rsync_src='mavlab-gpu:/home/pats/moth_json/'
    subprocess.call(['mkdir -p ' + source_folder ], shell=True)
    cmd = ['rsync -zva --timeout=3 --exclude \'*.jpg.*\' --exclude \'*.xml.*\' --exclude \'*.txt.*\' ' + rsync_src + ' '+ source_folder]
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

while True:
    download_jsons()
    print ('Downloaded jsons')
    cmd = ['~/code/pats/analysis/moth_watcher/fill_db_with_jsons.py -i ' + source_folder]
    result = subprocess.run(cmd, shell=True,stdout=subprocess.PIPE)
    print(result.returncode, result.stdout, result.stderr)
    download_renders_all()
    time.sleep(3600)