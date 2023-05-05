#!/usr/bin/env python3
import os
import glob
import json
import argparse
import socket
import logging
import subprocess
import shutil
# import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from dateutil.relativedelta import relativedelta
import lib_base as lb
from process_session import process_session
from retro_fix_size import retro_fix


def check_if_system_at_office():
    if socket.gethostname().lower().startswith('pats'):
        cmd = 'sudo nmcli dev wifi | grep PATS'
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        p_result = popen.stdout.readline().decode('utf-8')
        return bool(p_result)
    else:
        return False


def aggregate_jsons(data_folder, sys_str, aggregated_fn):
    retro_fix()

    Path(data_folder + '/processed').mkdir(parents=True, exist_ok=True)
    Path(data_folder + '/junk').mkdir(parents=True, exist_ok=True)
    Path(lb.json_dir).mkdir(parents=True, exist_ok=True)

    found_dirs = glob.glob(data_folder + "/202*_*")
    ordered_dirs = lb.natural_sort(found_dirs)

    detections = []
    statuss = []
    flights = []
    flight_sessions = []
    errors = []
    cam_resets = 0
    t_start = datetime.max
    t_end = datetime.min + relativedelta(years=1000)

    logger = logging.getLogger('aggregate_jsons')
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

    # move the folders so that we know if they were processed
    # moving is done after writing the json, so that if anything before that fails no data gets lost between the cracks
    for folder in ordered_dirs:
        top_folder = os.path.basename(folder)
        if os.path.exists(folder + '/OK'):
            processed_folder = folder[0:len(folder) - len(top_folder)] + 'processed/' + top_folder
            shutil.move(folder, processed_folder)
        elif os.path.exists(folder + '/junk'):
            junk_folder = folder[0:len(folder) - len(top_folder)] + 'junk/' + top_folder
            shutil.move(folder, junk_folder)
    logger.info("Data folders moved")


def send_all_jsons():
    logger = logging.getLogger('aggregate_jsons')
    Path(lb.json_dir + '/sent').mkdir(parents=True, exist_ok=True)
    files = glob.glob(lb.json_dir + '/*.tar.xz') + glob.glob(lb.json_dir + '/*.json')
    for json_fn in files:
        remote_json_file = 'jsons/' + os.path.basename(json_fn)
        cmd = 'rsync -a ' + json_fn + ' dash_upload:' + remote_json_file
        if lb.execute(cmd, 3, 'aggregate_jsons') == 0:
            json_sent_fn = lb.json_dir + '/sent/' + os.path.basename(json_fn)
            os.rename(json_fn, json_sent_fn)
            logger.info("Json sent: " + json_fn)
        else:
            return 1
    logger.info("Json sent to dash")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that counts the number of valid detections in a directory, bound by the minimum and maximum date.')
    parser.add_argument('-i', help="Path to the folder with logs", required=False)
    parser.add_argument('--dry-run', help="Dry run process system status just this one log", required=False)
    parser.add_argument('--filename', help="Path and filename to store results in", required=False)
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('aggregate_jsons')
    logger.setLevel(logging.DEBUG)

    if args.filename:
        out_fn = args.filename
    else:
        out_fn = lb.json_dir + socket.gethostname().lower() + '_' + lb.datetime_to_str_with_timezone(datetime.now())
    if args.i:
        data_folder = args.i
    else:
        data_dir = lb.data_dir

    aggregate_jsons(data_dir, socket.gethostname(), out_fn)

    if not args.dry_run:
        send_all_jsons()
