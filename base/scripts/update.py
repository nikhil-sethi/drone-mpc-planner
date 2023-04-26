#!/usr/bin/env python3

import subprocess
import os
import glob
import time
from datetime import datetime

# This script runs as super on all basestations. It downloads update scripts from dash and runs them as root.
# Very dangerous, so some annoying safeguards are in place. (e.g. the update folder must be root only, only admins can put updates on dash)
# This updater should only be used for system upgrades that require root. It is not ran from within the daemon.py.
# Daemon.py has a normal (non super user) updater called update_from_daemon.py.


def update_now():

    update_dir = '/home/pats/pats/updates/'
    if not os.path.exists(update_dir):
        print("Error: the update folder does not exist")
        return

    if not os.path.isfile('/home/pats/flags/disable_updates'):

        st = os.stat(update_dir)
        owner = st.st_uid
        permissions = st.st_mode & 0o777
        if owner != 0 or permissions != 493:
            print("Error: normal users have write access to the update folder")
            return

        rsync_cmd = ['rsync -aL -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR  -i /home/pats/.ssh/pats_upload_id_ed25519 -F /home/pats/pats/release/install/sshconfig" ' + 'dash_upload:updates/* ' + update_dir]
        subprocess.run(rsync_cmd, shell=True)

        update_files = sorted(glob.glob(update_dir + '*.update'), key=lambda x: x.split('/')[-1])
        for update_fn in update_files:
            done_fn = os.path.splitext(update_fn)[0] + '.done'
            if not os.path.exists(done_fn):
                with open(done_fn, "w") as f:
                    result = subprocess.run(update_fn, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    f.write(result.stdout.decode())
                    f.write(result.stderr.decode())


time.sleep(60)
try:
    update_now()
except Exception as e:  # pylint: disable=broad-except
    print(e)
updated_today = True

while True:

    now = datetime.now()
    if now.hour == 14 and not updated_today:
        updated_today = True
        try:
            update_now()
        except Exception as e:  # pylint: disable=broad-except
            print(e)
    elif now.hour != 11 and updated_today:
        updated_today = False
    else:
        time.sleep(300)
