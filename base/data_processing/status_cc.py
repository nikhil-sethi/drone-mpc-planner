#!/usr/bin/env python3
import os
import shutil
import datetime
import glob
import logging
import lib_base as lb
from pathlib import Path


def send_status_update():
    first_read = ''
    f = open(lb.local_status_txt_file, 'r')
    try:
        lines = f.readlines()
        if lines:
            if (lines[0] != first_read):
                first_read = lines[0]

                if os.path.exists(lb.local_xml_folder):
                    cmd = 'rsync -az ' + lb.local_xml_folder + ' dash:' + lb.remote_xml_folder
                    lb.execute(cmd, 1, 'status_cc')
                if os.path.exists(lb.local_pats_xml_override):
                    cmd = 'rsync -az ' + lb.local_pats_xml_override + ' dash:' + lb.remote_pats_xml_override
                    lb.execute(cmd, 1, 'status_cc')
                if os.path.exists(lb.local_status_txt_file):
                    cmd = 'rsync -az ' + lb.local_status_txt_file + ' dash:' + lb.remote_status_txt_file
                    lb.execute(cmd, 1, 'status_cc')
                if os.path.exists(lb.local_system_txt_file):
                    cmd = 'rsync -az ' + lb.local_system_txt_file + ' dash:' + lb.remote_system_txt_file
                    lb.execute(cmd, 1, 'status_cc')
                if os.path.exists(lb.local_status_im_file):
                    cmd = 'rsync -a ' + lb.local_status_im_file + ' dash:' + lb.remote_status_im_file
                    lb.execute(cmd, 1, 'status_cc')
    except:
        pass

    f.close()


if __name__ == "__main__":
    send_status_update()
