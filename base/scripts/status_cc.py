#!/usr/bin/env python3
import os
import lib_base as lb


def send_status_update():
    first_read = ''
    with open(lb.local_status_txt_file, 'r', encoding="utf-8") as f:
        try:
            lines = f.readlines()
            if lines:
                if lines[0] != first_read:
                    first_read = lines[0]

                    if os.path.exists(lb.local_status_txt_file):
                        cmd = 'rsync -az ' + lb.local_status_txt_file + ' dash_upload:' + lb.remote_status_txt_file
                        lb.execute(cmd, 1, 'status_cc')
                    if os.path.exists(lb.local_system_txt_file):
                        cmd = 'rsync -az ' + lb.local_system_txt_file + ' dash_upload:' + lb.remote_system_txt_file
                        lb.execute(cmd, 1, 'status_cc')
                    if os.path.exists(lb.local_status_im_file):
                        cmd = 'rsync -a ' + lb.local_status_im_file + ' dash_upload:' + lb.remote_status_im_file
                        lb.execute(cmd, 1, 'status_cc')
        except Exception:  # pylint: disable=broad-except
            pass


if __name__ == "__main__":
    send_status_update()
