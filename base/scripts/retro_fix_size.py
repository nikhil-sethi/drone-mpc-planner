#!/usr/bin/env python3
import os
import socket
import json
import glob
import re
import sys
import subprocess
import logging
from pathlib import Path
from datetime import datetime
import lib_base as lb
from process_session import process_session
from dateutil.relativedelta import relativedelta

# TMP fix for size bug https://github.com/pats-drones/pats/issues/1329


rotate_time = datetime(1, 1, 1, hour=9, minute=25)  # for the rotate time only hour and minute are used so year, month and day are irrelevant
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
error_file_handler = logging.handlers.WatchedFileHandler(filename=lb.daily_errs_log)  # a watched file handler makes sure that logging continues to the new file if it is rotated.
error_file_handler.setFormatter(file_format)
error_file_handler.level = logging.ERROR

logging.basicConfig()
logger = logging.getLogger('retro_fix_size')
logger.setLevel(logging.DEBUG)
logger.addHandler(error_file_handler)
logger.addHandler(logging.StreamHandler(sys.stdout))


def dir_to_datetime(dir_name):
    try:
        dir_date = datetime.strptime(os.path.basename(dir_name), "%Y%m%d_%H%M%S")
    except Exception:  # pylint: disable=broad-except
        dir_date = datetime(year=1, month=1, day=1)
    return dir_date


def check_if_system_at_office():
    if socket.gethostname().lower().startswith('pats'):
        cmd = 'sudo nmcli dev wifi | grep PATS'
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        p_result = popen.stdout.readline().decode('utf-8')
        return bool(p_result)
    else:
        return False


def aggregate_jsons(ordered_dirs, sys_str, aggregated_fn, logger):

    detections = []
    statuss = []
    flights = []
    flight_sessions = []
    errors = []
    cam_resets = 0
    t_start = datetime.max
    t_end = datetime.min + relativedelta(years=1000)

    for folder in ordered_dirs:
        logger.info('Processing: ' + folder)
        json_fn = folder + '/results.json'
        if not os.path.exists(json_fn):
            process_session(folder, logger)
        if not os.path.exists(json_fn):
            continue
        with open(json_fn) as json_file:
            try:
                data = json.load(json_file)

                start_datetime = lb.str_to_datetime(data['start_datetime'])
                end_datetime = lb.str_to_datetime(data['end_datetime'])
                if start_datetime < t_start:
                    t_start = start_datetime
                if end_datetime > t_end:
                    t_end = end_datetime

                if 'detections' in data:
                    detections.extend(data['detections'])
                if 'statuss' in data:
                    statuss.extend(data['statuss'])
                if 'flights' in data:
                    flights.extend(data['flights'])
                if 'flight_sessions' in data:
                    if len(data['flight_sessions']):
                        flight_sessions.append(data['flight_sessions'])
                errors.extend(data['errors'])
                cam_resets += data['cam_resets']
            except json.JSONDecodeError:
                logger.warning('Corrupted json file:' + json_fn)

    system_at_office = check_if_system_at_office()

    data_detections = {"start_datetime": lb.datetime_to_str(t_start),
                       "end_datetime": lb.datetime_to_str(t_end),
                       "system_at_office": system_at_office,
                       "detection_count": len(detections),
                       "detections": detections,
                       "flights": flights,
                       "flight_sessions": flight_sessions,
                       "mode": statuss,
                       "errors": errors,
                       "cam_resets": cam_resets,
                       "system": sys_str
                       }
    aggregated_json_fn = aggregated_fn + '.json'
    with open(aggregated_json_fn, 'w', encoding="utf-8") as outfile:
        json.dump(data_detections, outfile)

    aggregated_tar_fn = aggregated_fn + '.tar.xz'
    cmd = 'tar -C ' + os.path.split(aggregated_tar_fn)[0] + ' -cJf ' + aggregated_tar_fn + ' ' + os.path.split(aggregated_json_fn)[1]
    lb.execute(cmd, 1, 'aggregate_jsons')
    os.remove(aggregated_json_fn)

    logger.info("Counting complete, saved in " + aggregated_tar_fn)


def retro_fix():
    if not os.path.exists('/home/pats/dependencies/retro_fix_20230419.done'):
        found_dirs = sorted(glob.glob(lb.data_dir + "processed/202304*_*"), key=dir_to_datetime)
        found_dirs_fixed = []
        for dir_name in found_dirs:
            stripped_path = Path(dir_name, 'STRIPPED')
            if not os.path.exists(stripped_path):
                terminal_log_path = Path(dir_name, 'terminal.log')
                with open(terminal_log_path, 'r', encoding="utf-8") as file:
                    contents = file.read()
                    exe_name = '/home/pats/pats/release/tmp/retrofix-20 '
                    if "4e1e1dab0732b37133accdb6a9f43ac7798cc84a" in contents:
                        exe_name = '/home/pats/pats/release/tmp/retrofix-18 '
                        # lb.execute('./executor-18 --log ' + dir_name + ' ')
                        print("This folder needs fixing release 18 style")
                        found_dirs_fixed.append(dir_name)
                    elif "636884f7340ed45d70c39d65c7f3504f7d228aa7" in contents or 'c4b21f3647aeec76b6e929b61b460e10f8218135' in contents:
                        print("This folder needs fixing release 20 style")
                        found_dirs_fixed.append(dir_name)
                    else:
                        continue
            else:
                continue

            detection_fns = lb.natural_sort([fp for fp in glob.glob(os.path.join(dir_name, "log_i*.csv"))])
            if len(detection_fns):
                for detection_fn in detection_fns:
                    n = os.path.splitext(os.path.split(detection_fn)[1])[0][8:]
                    lb.execute(exe_name + '--log ' + dir_name + ' --insect ' + n)
                    if not os.path.exists(detection_fn + '.old'):
                        os.rename(detection_fn, detection_fn + '.old')
                    os.rename(detection_fn + '.fix', detection_fn)

                if os.path.exists(dir_name + '/results.json') and not os.path.exists(dir_name + '/results.json.old'):
                    os.rename(dir_name + '/results.json', dir_name + '/results.json.old')
                process_session(dir_name, logger, False)
                if os.path.exists(dir_name + '/results.json'):
                    with open(dir_name + '/results.json.old') as json_file_old:
                        data_old = json.load(json_file_old)
                    with open(dir_name + '/results.json') as json_file_new:
                        data_new = json.load(json_file_new)
                    for i in range(0, len(data_new['detections'])):
                        data_old['detections'][i]['size'] = data_new['detections'][i]['size']
                    with open(dir_name + '/results.json', 'w', encoding="utf-8") as json_file_fixed:
                        json.dump(data_old, json_file_fixed)
        aggregate_jsons(found_dirs_fixed, socket.gethostname(), lb.json_dir + 'retro_fix_' + lb.datetime_to_str_with_timezone(datetime.now()), logger=logger)
        with open('/home/pats/dependencies/retro_fix_20230419.done', 'w', encoding="utf-8") as touchy_file:
            touchy_file.write('OK')


retro_fix()
