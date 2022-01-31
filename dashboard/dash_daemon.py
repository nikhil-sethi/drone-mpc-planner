#!/usr/bin/env python3
import time
import argparse
import datetime
import logging
import logging.handlers
import sys
from pathlib import Path
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc
from jsons_to_db import jsons_to_db
from jsons_to_LG import jsons_to_LG


parser = argparse.ArgumentParser(description='Script that adds the json files to an sql database.')
parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/patsc/jsons/')
parser.add_argument('-p', '--period', help="Repeat this script every period", default=3600)
parser.add_argument('--dry-run', help="Run script now without processing data", dest='dry_run', action='store_true')
args = parser.parse_args()

Path(Path(patsc.daemon_error_log).parent.absolute()).mkdir(parents=True, exist_ok=True)
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
error_file_handler = logging.handlers.RotatingFileHandler(filename=patsc.daemon_error_log, maxBytes=1024 * 1024 * 100, backupCount=1)
error_file_handler.setFormatter(file_format)
error_file_handler.level = logging.ERROR

logger = logging.Logger('Dash_errors', level='ERROR')
logger.addHandler(error_file_handler)


def get_error_count():
    logger.error('rotate!')
    with open(patsc.daemon_error_log, 'r') as f:
        lines = f.readlines
        return len(lines) - 1


while True:
    try:
        jsons_to_db(args.input_folder, args.dry_run)
    except Exception as e:
        logger.error('JSONs to db error: ' + str(e))
    try:
        jsons_to_LG(args.input_folder, args.dry_run)
    except Exception as e:
        logger.error('JSONs to LG error: ' + str(e))

    if args.period:
        print(str(datetime.datetime.now()) + ". Periodic update after: " + str(args.period))
        time.sleep(int(args.period))
    else:
        break
