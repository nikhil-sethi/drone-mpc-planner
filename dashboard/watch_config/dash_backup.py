#!/usr/bin/env python3
import subprocess
import time
from datetime import datetime
from pytz import timezone


def execute(cmd):
    p_result = None

    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    for stdout_line in iter(popen.stdout.readline, ""):
        p_result = popen.poll()
        if p_result is not None:
            break
        print(stdout_line.decode('utf-8'), end='')
    popen.stdout.close()


def backup(now):
    from pathlib import Path
    Path("~/dash_backups/").expanduser().mkdir(parents=True, exist_ok=True)
    target_str = now.strftime('%Y%m%d_%H%M%S') + '.tar.gz'
    cmd = 'rsync -azvP dash:jsons/* ~/jsons/'
    execute(cmd)
    cmd = 'rsync -az dash:patsc/db/pats.db ~/patsc/db/'
    execute(cmd)
    cmd = 'rsync -az dash:patsc/db/pats_human_classification.db ~/patsc/db/'
    execute(cmd)
    cmd = 'rsync -az dash:patsc/db/.pats-c-key.py ~/patsc/db/'
    execute(cmd)
    cmd = 'rsync -az dash:patsc/db/pats_creds.db ~/patsc/db/'
    execute(cmd)
    cmd = 'rsync -az dash:patsc/db/pats_systems.db ~/patsc/db/'
    execute(cmd)
    cmd = 'tar -zcvf ~/dash_backups/' + target_str + ' ~/patsc/db/ ~/jsons'
    execute(cmd)
    return target_str


updated_today = False
cet = timezone('Europe/Amsterdam')
while True:
    now = datetime.now(cet)
    if (now.hour == int(10) and not updated_today):
        updated_today = True
        target_str = backup(now)
        print('Backed up to: ' + target_str)

    if now.hour == int(10) + 1 and updated_today:
        updated_today = False
    time.sleep(10)
