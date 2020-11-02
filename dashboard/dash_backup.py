#!/usr/bin/env python3
import socket, json,shutil,sqlite3,subprocess
import time, argparse
import pickle, glob, os, re
from datetime import datetime, timedelta

def execute(cmd):
    p_result = None

    popen = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE)
    for stdout_line in iter(popen.stdout.readline, ""):
        p_result = popen.poll()
        if p_result != None:
            break
        print(stdout_line.decode('utf-8'),end ='')
    popen.stdout.close()


def backup(now):
    from pathlib import Path
    Path("~/dash_backups/").expanduser().mkdir(parents=True, exist_ok=True)
    target_str = now.strftime('%Y%m%d_%H%M%S') + '.tar.gz'
    cmd = 'rsync -azvP dash:jsons/* ~/jsons/'
    execute(cmd)
    cmd = 'rsync -az dash:pats.db ~/'
    execute(cmd)
    cmd = 'tar -zcvf ~/dash_backups/' + target_str + ' ~/pats.db ~/jsons'
    execute(cmd)
    return target_str

updated_today = False
while  True:
    now = datetime.now()
    if now.hour == int(3) and not updated_today:
        updated_today = True
        target_str = backup(now)
        print('Backed up to: ' + target_str)

    if now.hour == int(3)+1 and updated_today:
        updated_today = False
    time.sleep(10)