#!/usr/bin/env python3
import os
import shutil
import glob
import logging
import lib_base as lb
from pathlib import Path
from datetime import datetime, timedelta


def clean_hd():
    logger = logging.getLogger('clean_hd')

    if not os.path.exists(lb.data_dir):
        os.mkdir(lb.data_dir)

    while True:
        total_used_space, _, free_space = shutil.disk_usage(lb.data_dir)
        logger.info('Free space: ' + str(free_space / 1024 / 1024 / 1024) + 'GB --> ' + str(round(free_space / total_used_space * 100)) + '% free')
        if (free_space / total_used_space < 0.2):
            if os.path.exists(lb.term_log_path):
                if Path(lb.term_log_path).stat().st_size > 1024 * 1024 * 1024:  # 1GB
                    os.remove(lb.term_log_path)
            found_dirs = sorted(glob.glob(lb.data_dir + "*/202*_*"), key=os.path.getmtime)
            for dir in found_dirs:
                try:
                    dir_date = datetime.strptime(os.path.basename(dir), "%Y%m%d_%H%M%S")
                except Exception:
                    logger.error('could not get date from: ' + dir)
                    logger.info('removing: ' + dir)
                    shutil.rmtree(dir)
                    break
                if ((datetime.now() - dir_date) > timedelta(days=14)):
                    logger.info('removing: ' + dir)
                    if os.path.exists(dir + '/logging'):
                        shutil.rmtree(dir)
                        break
                    shutil.rmtree(dir)
                else:
                    return
        else:
            return


if __name__ == "__main__":
    clean_hd()
