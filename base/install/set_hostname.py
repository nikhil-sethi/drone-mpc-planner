#!/usr/bin/env python3

import uuid
import subprocess
import os
import sys
from pathlib import Path

if not os.path.isfile('/home/pats/dependencies/hostname_set'):

    id_db_path = '/home/pats/pats/id_db.txt'
    rsync_cmd = ['rsync -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR  -i /home/pats/.ssh/id_rsa -F /home/pats/.ssh/config" ' + 'dash:id_db.txt ' + id_db_path]
    subprocess.run(rsync_cmd, shell=True)

    with open(id_db_path, 'r', encoding="utf-8") as sysdb:
        lines = sysdb.readlines()
        ser = subprocess.check_output("cat /sys/class/dmi/id/product_serial",shell=True).decode(sys.stdout.encoding).strip()
        for line in lines:
            if line.split(' ')[0] == ser:
                print('FOUND:' + line)
                pats_id = line.split(' ')[1]
                
                subprocess.run('hostnamectl set-hostname ' + pats_id, shell=True)

                with open('/home/pats/dependencies/hostname_set', 'w', encoding="utf-8") as touchy_file:
                    touchy_file.write(pats_id)
                exit(0)

    print('Error: product id not found')
    