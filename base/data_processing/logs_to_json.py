#!/usr/bin/env python3
import os
import glob
import json
import math
import argparse
import socket
import logging
import subprocess
import shutil
import sys
# import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime, timedelta
from dateutil.relativedelta import relativedelta
import numpy as np
import pandas as pd
import lib_base as lb
from lib_base import datetime_to_str, natural_sort, str_to_datetime
sys.path.append('ai')  # noqa
import use_model as ai
from cut_moths import cut_all


version_c = "2.1"
version_x = "1.1"
lia_model = "cnn5_size_v1"
n_insects = 5


def measured_exposure(terminal_log_path):
    daylight_start = ''
    daylight_end = ''
    prev_line = ''
    cnt = 0
    with open(terminal_log_path, "r", encoding="utf-8") as terminal_log:
        log_start = terminal_log.readline()
        while not daylight_end:
            line = terminal_log.readline()
            if 'Measured exposure' in line:
                daylight_start = line.split('Measured')[0]
                prev_line = line
                while True:
                    line = terminal_log.readline()
                    if 'Measured exposure' not in line:
                        daylight_end = prev_line.split('Measured')[0]
                        break
                    prev_line = line
                    cnt += 1
            if line == '' and prev_line == '':
                daylight_start = log_start
                daylight_end = log_start
                break

    daylight_start = daylight_start.strip().replace('/', '').replace(':', '').replace(' ', '_')
    daylight_end = daylight_end.strip().replace('/', '').replace(':', '').replace(' ', '_')

    if cnt < 5:
        return False, daylight_start, daylight_end
    else:
        return True, daylight_start, daylight_end


def process_wait_for_dark_status(folder):
    terminal_log_path = Path(folder, 'terminal.log')
    if os.path.exists(terminal_log_path):
        exposure_valid, daylight_start, daylight_end = measured_exposure(terminal_log_path)
        if exposure_valid:
            data_wait_for_dark = {"start_datetime": daylight_start,
                                  "end_datetime": daylight_end,
                                  "mode": 'wait_for_dark'
                                  }
            return (exposure_valid, data_wait_for_dark, daylight_start, daylight_end)
        return (False, [], daylight_start, daylight_end)
    return (False, [], '', '')


def process_system_status_in_folder(folder):
    logging.getLogger('logs_to_json').info('Processing status...')
    pats_xml_mode = ''
    pats_xml_path = Path(folder, 'pats.xml')
    terminal_log_path = Path(folder, 'terminal.log')
    if not os.path.exists(terminal_log_path):
        return ([], 'Error: terminal log do not exist', '')

    with open(terminal_log_path, "r", encoding="utf-8") as terminal_log:
        log_start = terminal_log.readline().strip()
    if log_start == 'Resetting cam':
        return ([], 'Resetting cam', '')
    if log_start.startswith('Error'):
        return ([], 'Error: ' + log_start, '')
    if log_start == '':
        return ([], 'Error: log_start empty!?', '')

    if not os.path.exists(pats_xml_path):
        line = subprocess.check_output(['tail', '-1', terminal_log_path]).decode("utf8").strip()
        return ([], 'Error: xml does not exist. Last terminal line: ' + line, '')
    with open(pats_xml_path, "r", encoding="utf-8") as pats_xml:
        xml_lines = pats_xml.readlines()
        for line in xml_lines:
            if line.find('op_mode') != -1:
                pats_xml_mode = line.split('_')[3].split('<')[0]
                break

    results_path = Path(folder, 'results.txt')
    if not os.path.exists(results_path):
        return ([], 'Error: results.txt does not exist', '')
    runtime = -1
    with open(results_path, "r", encoding="utf-8") as results_txt:
        result_lines = results_txt.readlines()
        for line in result_lines:
            if line.find('run_time') != -1:
                runtime = float(line.split(':')[1])
                break
    if runtime < 0:
        # something bad must have happened. A crash of the program prevents the writing of results.txt
        return ([], 'Error: results.txt does not contain run_time', '')

    log_end_datetime = str_to_datetime(os.path.basename(folder))
    operational_log_start = datetime_to_str(log_end_datetime - timedelta(seconds=runtime))

    data_status = {"start_datetime": operational_log_start,
                   "end_datetime": os.path.basename(folder),
                   "mode": pats_xml_mode
                   }
    return data_status, pats_xml_mode, operational_log_start


def process_flight_status_in_folder(folder, operational_log_start, mode):
    logging.getLogger('logs_to_json').info('Processing flight session status...')
    results_path = Path(folder, 'results.txt')
    if not os.path.exists(results_path):
        return []
    drone_flights = 0
    n_drone_detects = 0
    n_detections = 0
    n_takeoffs = 0
    n_landings = 0
    n_hunts = 0
    n_replay_hunts = 0
    with open(results_path, "r", encoding="utf-8") as results_txt:
        result_lines = results_txt.readlines()
        for line in result_lines:
            if line.find('n_drone_detects') != -1:
                n_drone_detects = int(line.strip().split(':')[1])
            if line.find('n_insects') != -1:
                n_detections = int(line.strip().split(':')[1])
            if line.find('n_takeoffs') != -1:
                n_takeoffs = int(line.strip().split(':')[1])
            if line.find('n_landings') != -1:
                n_landings = int(line.strip().split(':')[1])
            if line.find('n_hunts') != -1:
                n_hunts = int(line.strip().split(':')[1])
            if line.find('n_replay_hunts') != -1:
                n_replay_hunts = int(line.strip().split(':')[1])

    data = {"start_datetime": operational_log_start,
            "end_datetime": os.path.basename(folder),
            "drone_flights": drone_flights,
            "n_drone_detects": n_drone_detects,
            "n_insects": n_detections,
            "n_takeoffs": n_takeoffs,
            "n_landings": n_landings,
            "n_hunts": n_hunts,
            "n_replay_hunts": n_replay_hunts,
            "mode": mode,
            "version": version_x
            }

    return data


def process_flight_results(results_fn):
    logging.getLogger('logs_to_json').info('Processing flight results.txt...')

    flight_time = 0
    crashed = 0
    best_interception_distance = -1
    take_off_datetime = ''
    land_datetime = ''

    if os.path.exists(results_fn):
        with open(results_fn, "r", encoding="utf-8") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('flight_time') != -1:
                    flight_time = float(line.strip().split(':')[1])
                if line.find('crashed') != -1:
                    crashed = int(line.strip().split(':')[1])
                if line.find('best_interception_distance') != -1:
                    best_interception_distance = float(line.strip().split(':')[1])
                if line.find('take_off_datetime') != -1:
                    take_off_datetime = line.strip().split(':')[1]
                if line.find('land_datetime') != -1:
                    land_datetime = line.strip().split(':')[1]
    else:
        logging.getLogger('logs_to_json').warning('Flight results txt not found: ' + results_fn)

    return flight_time, crashed, best_interception_distance, take_off_datetime, land_datetime


def process_flight_log(log_fn, folder, session_start_datetime, mode):
    logger = logging.getLogger('logs_to_json')

    flight_id = int(os.path.basename(log_fn)[10:-4])  # assuming the name is log_flight**.csv. Maybe replace this with  https://stackoverflow.com/questions/14008440/how-to-extract-numbers-from-filename-in-python
    flight_time, crashed, best_interception_distance, _, _ = process_flight_results(folder + '/flight_results' + str(flight_id) + '.txt')

    if flight_time < 0:
        logger.info('Flight time invalid')
        return {}
    try:
        log = pd.read_csv(log_fn, sep=";")
    except Exception as e:  # pylint: disable=broad-except
        logger.info(log_fn + ': ' + str(e))
        return {}

    elapsed_time = log["elapsed"].values
    lost = log['n_frames_lost_drone'].values > 0
    remove_ids = [i for i, x in enumerate(lost) if x]
    if len(elapsed_time) < 20 or len(elapsed_time) - len(remove_ids) < 5:
        logger.info('Flight log too short')
        return {}

    vxs = log['svelX_drone'].values
    inf_ids = [i for i, x in enumerate(vxs) if math.isinf(x) or math.isnan(x)]
    if len(inf_ids):
        remove_ids.extend(inf_ids)
        logger.warning('Detected infs in velocity. See #540')

    rs_id = log['rs_id'].values

    filtered_elepased = np.delete(elapsed_time, remove_ids)
    start = filtered_elepased[0]
    end = filtered_elepased[-1]
    duration = end - start
    first_rs_id = str(rs_id[0])
    filename = os.path.basename(log_fn)
    video_filename = os.path.dirname(log_fn) + '/' + filename.replace('log_flight', 'flight').replace('csv', 'mkv')
    if os.path.exists(video_filename) and duration > 0.5:
        if os.stat(video_filename).st_size > 30000:
            render_tag_filename = os.path.dirname(log_fn) + '/' + filename.replace('log_flight', 'flight').replace('csv', 'render_tag')
            Path(render_tag_filename).touch()
            video_filename = os.path.basename(video_filename)
        else:
            logger.warning("video file doesn't have many bytes" + str(os.stat(video_filename).st_size))
            video_filename = 'NA; video corrupted'
    elif not os.path.exists(video_filename):
        logger.warning("video file source does not exist: " + video_filename)
        video_filename = 'NA; video not available'
    else:
        logger.info("flight too short to render video: " + str(duration))
        video_filename = 'NA; flight too short'

    takeoff_time = datetime_to_str(session_start_datetime + timedelta(seconds=elapsed_time[0]))

    flight_data = {"start_datetime": takeoff_time,
                   "duration": duration,
                   "flight_time": flight_time,
                   "best_interception_distance": best_interception_distance,
                   "crashed": crashed,
                   "rs_id": first_rs_id,
                   "detection_ids": str(log['insect_id'].unique()),
                   "filename": filename,
                   "folder": os.path.basename(folder),
                   "video_filename": video_filename,
                   "version": version_x
                   }
    return flight_data


def process_detection_log(log_fn, folder, mode, session_start_datetime):
    logger = logging.getLogger('logs_to_json')
    try:
        log = pd.read_csv(log_fn, sep=";", dtype=float, converters={'fp': str})
    except Exception as e:  # pylint: disable=broad-except
        logger.info(log_fn + ': ' + str(e))
        return {}

    elapsed_time = log["elapsed"].values
    lost = log['n_frames_lost_insect'].values > 0
    remove_ids = [i for i, x in enumerate(lost) if x]
    if len(elapsed_time) < 20 or len(elapsed_time) - len(remove_ids) < 5:
        return {}

    # there can be infs in the velocity columns, due to some weird double frameid occurance from the realsense. #540
    vxs = log['svelX_insect'].values
    vys = log['svelY_insect'].values
    vzs = log['svelZ_insect'].values
    inf_ids = [i for i, x in enumerate(vxs) if math.isinf(x) or math.isnan(x)]
    if len(inf_ids):
        remove_ids.extend(inf_ids)
        logger.warning('Detected infs in velocity. See #540')

    rs_id = log['rs_id'].values

    xs = log['sposX_insect'].values
    ys = log['sposY_insect'].values
    zs = log['sposZ_insect'].values
    x_tracking = np.delete(xs, remove_ids).astype('float64')
    y_tracking = np.delete(ys, remove_ids).astype('float64')
    z_tracking = np.delete(zs, remove_ids).astype('float64')

    pos_start = [x_tracking[0], y_tracking[0], z_tracking[0]]
    pos_end = [x_tracking[-1], y_tracking[-1], z_tracking[-1]]
    diff = np.array(pos_end) - np.array(pos_start)
    dist_traveled = math.sqrt(np.sum(diff * diff))
    dist_traject = np.sum(np.sqrt((x_tracking[1:] - x_tracking[:-1])**2 + (y_tracking[1:] - y_tracking[:-1])**2 + (z_tracking[1:] - z_tracking[:-1])**2))

    mid_id = int(round(len(x_tracking) / 2))
    pos_middle = [x_tracking[mid_id], y_tracking[mid_id], z_tracking[mid_id]]
    alpha_horizontal_start = math.atan2(pos_middle[0] - pos_start[0], pos_middle[2] - pos_start[2])
    alpha_horizontal_end = math.atan2(pos_end[0] - pos_middle[0], pos_end[2] - pos_middle[2])
    alpha_vertical_start = math.atan2(pos_middle[0] - pos_start[0], pos_middle[1] - pos_start[1])
    alpha_vertical_end = math.atan2(pos_end[0] - pos_middle[0], pos_end[1] - pos_middle[1])

    vx_tracking = np.delete(vxs, remove_ids).astype('float64')
    vy_tracking = np.delete(vys, remove_ids).astype('float64')
    vz_tracking = np.delete(vzs, remove_ids).astype('float64')
    v = np.sqrt(vx_tracking**2 + vy_tracking**2 + vz_tracking**2)
    v_mean = v.mean()
    v_std = v.std()

    insect_chances = ai.use_the_model("cnn", log, amount_classes=n_insects, restrict_var=0)  # hardcoded which model to use

    if insect_chances:
        average_insect_chance = np.mean(np.array(insect_chances), axis=0,).flatten()
        pred_insect_from_trajectory = int(np.argmax(average_insect_chance))
    else:
        average_insect_chance = np.zeros((5, 1))
        pred_insect_from_trajectory = -1

    radiuss = np.delete(log['radius_insect'].values, remove_ids)
    size = np.mean(radiuss) * 2

    if 'motion_sum_insect' in log:
        motion_sums = np.delete(log['motion_sum_insect'].values, remove_ids)
        d_motion_sums = motion_sums[1:] - motion_sums[:-1]

        ps = np.abs(np.fft.rfft(d_motion_sums))
        ps_max_id = np.argmax(ps)
        freqs = np.fft.rfftfreq(d_motion_sums.size, 1 / 90)
        wing_beat = freqs[ps_max_id]
        # idx = np.argsort(freqs)
        # plt.plot(freqs[idx], ps[idx]**2)
        # plt.show()
    else:
        wing_beat = -1

    monster = False
    if 'fp' in log:
        fps = log['fp'].values
        if fps[-1] == 'fp_too_big' or fps[-1] == 'fp_too_far':
            monster = True

    hunt_id = int(0)
    if 'hunt_id' in log:
        hunt_id = int(log['hunt_id'].values[-1])

    filtered_elepased = np.delete(elapsed_time, remove_ids)
    start = filtered_elepased[0]
    end = filtered_elepased[-1]
    duration = end - start
    first_rs_id = str(rs_id[0])
    filename = os.path.basename(log_fn)
    video_filename = os.path.dirname(log_fn) + '/' + filename.replace('log_itrk', 'insect').replace('csv', 'mkv')
    if os.path.exists(video_filename) and duration > 1 and duration < 10:  # this filter is also used in PATS-C
        if os.stat(video_filename).st_size > 30000:
            render_tag_filename = os.path.dirname(log_fn) + '/' + filename.replace('log_itrk', 'insect').replace('csv', 'render_tag')
            Path(render_tag_filename).touch()
            video_filename = os.path.basename(video_filename)
        else:
            logger.warning("video file doesn't have many bytes" + str(os.stat(video_filename).st_size))
            video_filename = 'NA; video corrupted'
    elif not os.path.exists(video_filename):
        logger.warning("video file source does not exist: " + video_filename)
        video_filename = 'NA; video not available'
    else:
        logger.info("detection too short to render video: " + str(duration))
        video_filename = 'NA; detection too short'

    detection_time = datetime_to_str(session_start_datetime + timedelta(seconds=elapsed_time[0]))

    detection_data = {"start_datetime": detection_time,
                      "duration": duration,
                      "rs_id": first_rs_id,
                      "dist_traveled": dist_traveled,
                      "dist_traject": dist_traject,
                      "size": size,
                      "wing_beat": wing_beat,
                      "vel_mean": v_mean,
                      "vel_std": v_std,
                      "vel_max": v.max(),
                      "lia_insect": pred_insect_from_trajectory,
                      "lia_version": lia_model,
                      "chance_chrysodeixis_chalcites": float(average_insect_chance[0]),
                      "chance_duponchelia_fovealis": float(average_insect_chance[1]),
                      "chance_plutella_xylostella": float(average_insect_chance[2]),
                      "chance_tuta_absoluta": float(average_insect_chance[3]),
                      "chance_opogona_sacchari": float(average_insect_chance[4]),
                      "alpha_horizontal_start": alpha_horizontal_start,
                      "alpha_horizontal_end": alpha_horizontal_end,
                      "alpha_vertical_start": alpha_vertical_start,
                      "alpha_vertical_end": alpha_vertical_end,
                      "filename": filename,
                      "folder": os.path.basename(folder),
                      "video_filename": video_filename,
                      "monster": monster,
                      "hunt_id": hunt_id,
                      "version": version_c
                      }
    return detection_data


def process_flights_in_folder(folder, operational_log_start, mode):
    logger = logging.getLogger('logs_to_json')
    flight_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_f*.csv"))])
    start_datetime = str_to_datetime(operational_log_start)

    session_data = []

    for flight_fn in flight_fns:
        logger.info("Processing flights in " + flight_fn)
        data = process_flight_log(flight_fn, folder, start_datetime, mode)
        if data:
            session_data.append(data)

    return session_data


def process_detections_in_folder(folder, operational_log_start, mode):
    logger = logging.getLogger('logs_to_json')
    detection_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_i*.csv"))])
    session_start_datetime = str_to_datetime(operational_log_start)

    session_data = []
    for detection_fn in detection_fns:
        logger.info("Processing detections in " + detection_fn)
        data = process_detection_log(detection_fn, folder, mode, session_start_datetime)
        if data:
            session_data.append(data)

    return session_data


def check_if_system_at_office():
    cmd = 'sudo nmcli dev wifi | grep PATS'
    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    p_result = popen.stdout.readline().decode('utf-8')
    return not p_result


def logs_to_json(json_fn, data_folder, sys_str):

    Path(data_folder + '/processed').mkdir(parents=True, exist_ok=True)
    Path(data_folder + '/junk').mkdir(parents=True, exist_ok=True)
    Path(lb.json_dir).mkdir(parents=True, exist_ok=True)

    found_dirs = glob.glob(data_folder + "/202*_*")
    ordered_dirs = natural_sort(found_dirs)

    detections = []
    statuss = []
    flights = []
    flight_sessions = []
    errors = []
    cam_resets = 0
    t_start = datetime.max
    t_end = datetime.min + relativedelta(years=1000)

    logger = logging.getLogger('logs_to_json')
    for folder in ordered_dirs:
        logger.info("Processing " + folder)
        if os.path.exists(folder + '/videoRawLR.avi'):  # this folder was probably not yet processed in cut_moth.
            cut_all(folder)

        top_folder = os.path.basename(folder)
        t_folder = str_to_datetime(top_folder)
        if t_folder > t_end:
            t_end = t_folder

        exposure_valid, data_wait_for_dark, daylight_start, _ = process_wait_for_dark_status(folder)
        if exposure_valid:
            statuss.append(data_wait_for_dark)
            if lb.str_to_datetime(daylight_start) < t_start:
                t_start = lb.str_to_datetime(daylight_start)

        status_in_folder, mode, operational_log_start = process_system_status_in_folder(folder)

        if status_in_folder != []:
            statuss.append(status_in_folder)
            if lb.str_to_datetime(operational_log_start) < t_start:
                t_start = lb.str_to_datetime(operational_log_start)

        if mode.startswith('Error:'):
            errors.append(mode + '(' + folder + ')')
        elif mode.startswith('Resetting cam'):
            cam_resets += 1
        else:
            detections_in_folder = process_detections_in_folder(folder, operational_log_start, mode)
            if detections_in_folder != []:
                detections.extend(detections_in_folder)
            if mode == 'x':
                flight_status_in_folder = process_flight_status_in_folder(folder, operational_log_start, mode)
                if flight_status_in_folder != []:
                    flight_sessions.append(flight_status_in_folder)
                flights_in_folder = process_flights_in_folder(folder, operational_log_start, mode)
                if flights_in_folder != []:
                    flights.extend(flights_in_folder)

        if mode == 'c' or mode == 'x':
            Path(folder + '/OK').touch()
        else:
            with open(folder + '/junk', 'w', encoding="utf-8") as outfile:
                outfile.write(mode + '\n')

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
                       "system": sys_str,
                       "version": version_c}
    with open(json_fn, 'w', encoding="utf-8") as outfile:
        json.dump(data_detections, outfile)
    logger.info("Counting complete, saved in " + json_fn)

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


def process_all_logs_to_jsons():
    now = datetime.today()
    local_json_file = lb.json_dir + datetime_to_str(now) + '.json'
    logs_to_json(local_json_file, lb.data_dir, socket.gethostname())


def send_all_jsons():
    logger = logging.getLogger('logs_to_json')
    Path(lb.json_dir + '/sent').mkdir(parents=True, exist_ok=True)
    for json_fn in glob.glob(lb.json_dir + '/*.json'):
        remote_json_file = 'patsc/jsons/' + socket.gethostname() + '_' + os.path.basename(json_fn)
        cmd = 'rsync -az ' + json_fn + ' dash:' + remote_json_file
        if lb.execute(cmd, 3, 'logs_to_json') == 0:
            json_sent_fn = lb.json_dir + '/sent/' + os.path.basename(json_fn)
            os.rename(json_fn, json_sent_fn)
            logger.info("Json sent: " + json_fn)
        else:
            return 1
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that counts the number of valid detections in a directory, bound by the minimum and maximum date.')
    parser.add_argument('-i', help="Path to the folder with logs", required=False, default=lb.data_dir)
    parser.add_argument('--dry-run', help="Dry run process system status just this one log", required=False)
    parser.add_argument('--filename', help="Path and filename to store results in", default="./detections.json")
    parser.add_argument('--system', help="Override system name", default=socket.gethostname())
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('logs_to_json')
    logger.setLevel(logging.DEBUG)

    json_fn = args.filename
    data_folder = args.i
    sys_str = args.system

    if args.dry_run:
        process_all_logs_to_jsons()
    else:
        process_all_logs_to_jsons()
        send_all_jsons()
