import os
import time
import subprocess


if os.path.isfile('/home/pats/dependencies/hostname_set') and not os.path.isfile('/home/pats/dependencies/timezone_set'):
    time.sleep(5)  # seems to be necessary with old ubuntu 18 images, need some time to get wifi

    with open('/home/pats/dependencies/hostname_set', 'r') as f:
        system = f.readline().strip()

    cmd = ['ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -i /home/pats/.ssh/id_rsa -F /home/pats/pats/release/install/sshconfig -T dash /usr/bin/python3 < /home/pats/pats/release/scripts/run_timezone_loader.py - --system ' + system]
    subprocess.run(cmd, shell=True)

    time.sleep(5)  # dash needs some time to send us our timezone

    timezone_file = 'patsc/timezone/' + 'tmp_' + system
    target_file = '/home/pats/dependencies/timezone_set'
    cmd = ['rsync -a -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null  -i /home/pats/.ssh/id_rsa -F /home/pats/.ssh/config" dash:' + timezone_file + ' ' + target_file]
    subprocess.run(cmd, shell=True)

    if os.path.isfile('/home/pats/dependencies/timezone_set'):
        with open('/home/pats/dependencies/timezone_set', 'r') as f:
            timezone = f.readline().strip()

        try:
            cmd = ['/usr/bin/timedatectl set-timezone ' + timezone]
            subprocess.run(cmd, shell=True)
        except Exception as e:
            print(str(e))
            print('setting timezone failed, removing timezone file')
            os.remove('/home/pats/dependencies/timezone_set')

    else:
        print('Did not recieve timezone file')
