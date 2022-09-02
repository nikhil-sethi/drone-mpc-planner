#!/usr/bin/env python3
from cmath import isinf
from genericpath import exists
import os
import glob
import json
import math
import argparse
import logging
import subprocess
import sys
# import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
import lib_base as lb
from lib_base import datetime_to_str, natural_sort, str_to_datetime
sys.path.append('ai')  # noqa
try:
    __import__('torch')
except ImportError:
    torch_exists = False
else:
    torch_exists = True
    import use_model as ai


version_c = "2.3"
version_x = "1.2"
lia_model = "cnn5_size_v1"
lia_n_insect_classes = 5


def process_wait_for_start_condition_status(folder: str, logger: logging.Logger):
    logger.info('Processing wait_for_start...')
    wait_for_start_log_path = Path(folder, 'wait_for_start.csv')

    if not os.path.exists(wait_for_start_log_path):
        return (False, [], '', '')

    try:
        log = pd.read_csv(wait_for_start_log_path, sep=";")
    except pd.errors.EmptyDataError:
        logger.warning('pd.errors.EmptyDataError in wait_for_start log...')
        return (False, [], '', '')

    if len(log) < 5:
        return (False, [], '', '')

    datetimes = log['Datetime'].dropna()
    offline_start = datetimes.values[0]
    offline_end = datetimes.values[-1]

    offline_start = offline_start.strip().replace('/', '').replace(':', '').replace(' ', '_')
    offline_end = offline_end.strip().replace('/', '').replace(':', '').replace(' ', '_')

    data_wait_for_start = {"start_datetime": offline_start,
                           "end_datetime": offline_end,
                           "mode": 'wait_for_conditions'
                           }
    return (True, data_wait_for_start, offline_start, offline_end)


def process_system_status_in_folder(folder: str, logger: logging.Logger):
    logger.info('Processing status...')
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

    if os.path.exists(Path(folder, 'no_multimodule_flag')):
        return ([], 'Error: no multimodule:', '')

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
        logger.warning("Results.txt does not exist. Log was likely not closed properly...")
        frames_path = Path(folder, 'frames.csv')
        if not os.path.exists(frames_path):
            return ([], 'Error: Could not determine runtime', '')
        frames_log = pd.read_csv(frames_path, sep=";")
        if len(frames_log) > 2:
            runtime = frames_log['time'].values[-2]  # use the second last because a not properly closed log likely has a corrupted last line...
        else:
            runtime = 0
    else:
        runtime = -1.0
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

    session = {"start_datetime": operational_log_start,
               "end_datetime": os.path.basename(folder),
               "mode": pats_xml_mode
               }
    return session, pats_xml_mode, operational_log_start


def process_flight_status_in_folder(folder: str, operational_log_start: str, mode: str, logger: logging.Logger):
    logger.info('Processing flight session status...')
    results_path = Path(folder, 'results.txt')
    if not os.path.exists(results_path):
        return {}
    drone_flights = 0
    n_drone_detects = 0
    drone_has_been_ready = False
    n_detections = 0
    n_takeoffs = 0
    n_landings = 0
    n_hunts = 0
    n_replay_hunts = 0
    with open(results_path, "r", encoding="utf-8") as results_txt:
        result_lines = results_txt.readlines()
        for line in result_lines:
            if line.find('drone_has_been_ready') != -1:
                drone_has_been_ready = bool(line.strip().split(':')[1])
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

    if str_to_datetime(os.path.basename(folder)) - str_to_datetime(operational_log_start) < timedelta(seconds=60):
        return {}

    session = {"start_datetime": operational_log_start,
               "end_datetime": os.path.basename(folder),
               "drone_has_been_ready": drone_has_been_ready,
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

    return session


def process_flight_results(results_fn: str, logger: logging.Logger):
    logger.info('Processing flight results.txt...')

    flight_time = 0.0
    crashed = 0
    best_interception_distance = -1.0
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
                    if isinf(best_interception_distance):
                        best_interception_distance = -1
                if line.find('take_off_datetime') != -1:
                    take_off_datetime = line.strip().split(':')[1]
                if line.find('land_datetime') != -1:
                    land_datetime = line.strip().split(':')[1]
    else:
        logging.getLogger('process_session').warning('Flight results txt not found: ' + results_fn)

    return flight_time, crashed, best_interception_distance, take_off_datetime, land_datetime


def process_flight_log(log_fn: str, folder: str, session_start_datetime: datetime, logger: logging.Logger):
    flight_id = int(os.path.basename(log_fn)[10:-4])  # assuming the name is log_flight**.csv. Maybe replace this with  https://stackoverflow.com/questions/14008440/how-to-extract-numbers-from-filename-in-python
    flight_time, crashed, best_interception_distance, _, _ = process_flight_results(folder + '/flight_results' + str(flight_id) + '.txt', logger)

    if flight_time <= 0:
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
    start = int(rs_id[0])
    end = int(rs_id[-2])
    duration = (end - start) / 90
    first_rs_id = int(rs_id[0])
    filename = os.path.basename(log_fn)

    takeoff_time = datetime_to_str(session_start_datetime + timedelta(seconds=elapsed_time[0]))

    unique_detection_ids = log['insect_id'].dropna().unique()
    unique_detection_ids = np.delete(unique_detection_ids, np.where(unique_detection_ids <= 0))

    flights = {"start_datetime": takeoff_time,
               "duration": duration,
               "flight_id": flight_id,
               "flight_time": flight_time,
               "best_interception_distance": best_interception_distance,
               "crashed": crashed,
               "rs_id": first_rs_id,
               "detection_ids": str(unique_detection_ids).replace('.', ',').replace('[', '').replace(']', ''),
               "filename": filename,
               "folder": os.path.basename(folder),
               "version": version_x
               }
    return flights


def process_detection_log(log_fn: str, folder: str, session_start_datetime: datetime, flights_in_folder: list, logger: logging.Logger):
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

    folder_detection_id = int(os.path.basename(log_fn)[8:-4])  # assuming the name is log_itrk**.csv

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

    if torch_exists:
        insect_chances = ai.use_the_model("cnn", log, amount_classes=lia_n_insect_classes, restrict_var=0)  # hardcoded which model to use
    else:
        insect_chances = None

    if insect_chances:
        average_insect_chance = np.mean(np.array(insect_chances), axis=0,).flatten()
        pred_insect_from_trajectory = int(np.argmax(average_insect_chance))
    else:
        average_insect_chance = np.zeros((5, 1))
        pred_insect_from_trajectory = -1

    radiuss = np.delete(log['radius_insect'].values, remove_ids)
    size = np.mean(radiuss) * 2

    light_level = -1
    if 'light_level' in log:
        light_level = np.mean(log['light_level'].values)

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

    monster = 0
    if 'fp' in log:
        fps = log['fp'].dropna().values
        if fps[-1] == 'fp_too_big' or fps[-1] == 'fp_too_far':
            monster = 1

    if monster == 0:
        imLx = np.delete(log['imLx_insect'].values, remove_ids)
        imLy = np.delete(log['imLy_insect'].values, remove_ids)
        tot_std = np.std(imLx) + np.std(imLy)
        if tot_std < 10:
            monster = 3

    hunt_id = -1
    if 'hunt_id' in log:
        hunt_id_df = log['hunt_id'].dropna()
        hunt_id_df = hunt_id_df.drop(hunt_id_df[hunt_id_df <= 0].index)
        if not hunt_id_df.empty:
            hunt_id = hunt_id_df.groupby(hunt_id_df).size().agg(['idxmax', 'max'])[0]
            drone_flight = list(filter(lambda item: item['flight_id'] == hunt_id, flights_in_folder))
            if not drone_flight:
                hunt_id = -1

    filtered_elepased = np.delete(elapsed_time, remove_ids)
    start = filtered_elepased[0]
    end = filtered_elepased[-1]
    duration = end - start
    first_rs_id = int(rs_id[0])
    filename = os.path.basename(log_fn)

    detection_time = datetime_to_str(session_start_datetime + timedelta(seconds=elapsed_time[0]))

    detections = {"start_datetime": detection_time,
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
                  "folder_detection_id": folder_detection_id,
                  "monster": monster,
                  "hunt_id": hunt_id,
                  "light_level": light_level,
                  "version": version_c
                  }
    return detections


def process_flights_in_folder(folder: str, operational_log_start: str, logger: logging.Logger):
    flight_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_f*.csv"))])
    start_datetime = str_to_datetime(operational_log_start)

    sessions = []

    for flight_fn in flight_fns:
        logger.info("Processing flight " + flight_fn)
        data = process_flight_log(flight_fn, folder, start_datetime, logger)
        if data:
            sessions.append(data)

    return sessions


def concat_monsters(events: list, min_in_between_time: float):
    duration = -1
    for i in range(0, len(events)):
        if events[i]['duration'] > 0:
            end_rs_id = events[i]['rs_id'] + events[i]['duration'] * 90
            duration = events[i]['duration']
            for ii in range(i + 1, len(events)):
                if events[ii]['rs_id'] - (events[i]['rs_id'] + duration * 90) < 90 * min_in_between_time:
                    end_rs_id_ii = events[ii]['rs_id'] + events[ii]['duration'] * 90
                    if end_rs_id < end_rs_id_ii:
                        end_rs_id = end_rs_id_ii
                        duration = (events[ii]['rs_id'] - events[i]['rs_id']) / 90 + events[ii]['duration']
                    events[ii]['duration'] = -1
                else:
                    break
        if duration > 0:
            rs_id = events[i]['rs_id'] - 90
            if rs_id < 0:
                rs_id = 0
            duration += 2

            events[i]['duration'] = duration

            duration = -1


def process_detections_in_folder(folder: str, operational_log_start: str, flights_in_folder: list, logger: logging.Logger):
    detection_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "log_i*.csv"))])
    session_start_datetime = str_to_datetime(operational_log_start)

    detections = []
    for detection_fn in detection_fns:
        logger.info("Processing detection " + detection_fn)
        data = process_detection_log(detection_fn, folder, session_start_datetime, flights_in_folder, logger)
        if data:
            detections.append(data)

    monsters = [d for d in detections if d['monster'] == 1 and d['hunt_id'] < 0]
    concat_monsters(monsters, 10)
    detections = [d for d in detections if d['duration'] > 0]

    return detections


def create_and_associate_hunt_videos(flights, detections):
    hunt_cuts = []
    for flight in flights:
        start_rs_id = flight['rs_id']
        duration = flight['duration']
        if len(flight['detection_ids'][0]):
            trigger_insect_id = int(flight['detection_ids'].split(',')[0])
            for detection in detections:
                if detection['folder_detection_id'] == trigger_insect_id:
                    if start_rs_id > detection['rs_id']:  # if the insect was detected before takeoff, which it should have...
                        duration = (start_rs_id - detection['rs_id']) / 90 + flight['duration']
                        start_rs_id = detection['rs_id']
                    break
        start_rs_id = start_rs_id - 3 * 90
        if start_rs_id < 0:
            start_rs_id = 0
        duration += 6
        flight['video_duration'] = duration
        flight['video_filename'] = 'flight' + str(flight['flight_id']) + '.mkv'
        hunt_cuts.append({'rs_id': start_rs_id,
                          'end_rs_id': round(start_rs_id + 90 * duration),
                          'duration': duration,
                          'type': 'flight',
                          'id': flight['flight_id'],
                          'video_filename': flight['video_filename']
                          })
    return hunt_cuts


def create_concated_cuts(events: list, min_in_between_time: float, type_str: str):
    cuts = []
    duration = -1
    for i in range(0, len(events)):
        if 'concat_id' not in events[i]:
            end_rs_id = events[i]['rs_id'] + events[i]['duration'] * 90
            duration = events[i]['duration']
            for ii in range(i + 1, len(events)):
                if events[ii]['rs_id'] - (events[i]['rs_id'] + duration * 90) < 90 * min_in_between_time:
                    end_rs_id_ii = events[ii]['rs_id'] + events[ii]['duration'] * 90
                    if end_rs_id < end_rs_id_ii:
                        end_rs_id = end_rs_id_ii
                        duration = (events[ii]['rs_id'] - events[i]['rs_id']) / 90 + events[ii]['duration']
                    events[ii]['concat_id'] = events[i]['folder_detection_id']
                    events[ii]['video_filename'] = type_str + str(events[ii]['concat_id']) + '.mkv'
                else:
                    break
        if duration > 0:
            rs_id = events[i]['rs_id'] - 90
            if rs_id < 0:
                rs_id = 0
            duration += 2
            events[i]['video_duration'] = duration
            events[i]['video_filename'] = type_str + str(events[i]['folder_detection_id']) + '.mkv'
            cuts.append({'rs_id': events[i]['rs_id'],
                         'end_rs_id': round(events[i]['rs_id'] + 90 * duration),
                         'duration': duration,
                         'type': type_str,
                         'id': events[i]['folder_detection_id'],
                         'video_filename': events[i]['video_filename']
                         })

            duration = -1
    return cuts


def associate_insects_to_monsters_cuts(monster_cuts: list, insects: list, monster_in_between_time: float):
    cut_id = 0
    for insect in insects:
        for i in range(cut_id, len(monster_cuts)):
            if insect['rs_id'] >= monster_cuts[i]['rs_id'] - monster_in_between_time * 90 and insect['rs_id'] <= monster_cuts[i]['rs_id'] + monster_cuts[i]['duration'] * 90 + monster_in_between_time * 90:
                insect['concat_id'] = monster_cuts[i]['id']
                insect['video_filename'] = monster_cuts[i]['video_filename']
                insect['monster'] = 2
                break
            elif insect['rs_id'] >= monster_cuts[i]['rs_id'] + monster_cuts[i]['duration'] * 90 + monster_in_between_time * 90:
                cut_id = i + 1


def apply_hunt_video_filename(d: dict):
    d['video_filename'] = 'flight' + str(d['hunt_id']) + '.mkv'


def associate_detections_to_flight_cuts(cuts: list, detections: list):
    [apply_hunt_video_filename(d) for d in detections if d['hunt_id'] >= 0]

    cuts_new = []
    for i in range(0, len(cuts) - 1):
        if cuts[i]['rs_id'] >= 0:
            for ii in range(i + 1, len(cuts)):
                if cuts[i]['end_rs_id'] > cuts[ii]['end_rs_id'] and (cuts[ii]['type'] == 'insect' or cuts[ii]['type'] == 'anomoly') and cuts[i]['type'] == 'flight':
                    for detection in detections:
                        if detection['video_filename'] == cuts[ii]['video_filename']:
                            detection.pop('concat_id', None)
                            detection.pop('video_duration', None)
                            detection['video_filename'] = cuts[i]['video_filename']
                    cuts[ii]['rs_id'] = -1
            cuts_new.append(cuts[i])
    if len(cuts) > 1:
        if cuts[i + 1]['rs_id'] >= 0:
            cuts_new.append(cuts[i + 1])
    return cuts_new


def run_ffmpeg(cuts, frames_fn, folder, video_in_fn):
    with open(frames_fn, 'r', encoding="utf-8") as frames_log:
        heads = frames_log.readline().rstrip().split(';')
        cut_log_fn = folder + '/cut.log'
        with open(cut_log_fn, 'w', encoding="utf-8") as cut_log:
            ffmpeg_cmd_init = 'ffmpeg -y -i ' + video_in_fn
            ffmpeg_cmd = ffmpeg_cmd_init
            arg_cnt = 0
            cut_log.write('Video file size: ' + str(os.path.getsize(video_in_fn) / 1024 / 1024 / 1024) + 'GB\n')
            key_frame_ids = []
            for cut in cuts:
                video_start_rs_id = cut['rs_id']
                video_start_time = -1
                video_start_video_id = -1
                while True:
                    frame_line = frames_log.readline()
                    splitted_frame_line = frame_line.rstrip().split(';')

                    if not frame_line or len(splitted_frame_line) != len(heads):
                        cut_log.write('Error: could not find frame in frames.csv\n')
                        print('Error: could not find frame in frames.csv')
                        break
                    frame = dict(zip(heads, splitted_frame_line))

                    if (int(frame['encoded_img_count'])) % 30 == 0:  # assuming a keyframe every 30 frames
                        key_frame_ids.append([int(frame['encoded_img_count']), int(frame['rs_id'])])

                    if int(frame['rs_id']) == video_start_rs_id:
                        keyframe_rs_id = key_frame_ids[-1][1]
                        video_start_video_id = key_frame_ids[-1][0]  # instead of using exactly int(frame['encoded_img_count']), the way the video encoding works is that we can only cut from the last keyframe
                        video_start_time = float(video_start_video_id - 1) / 90  # for some reason ffmpeg wants a time instead of a frame id. And it needs to be one frame before the actual key frame...
                        break
                    if int(frame['rs_id']) > video_start_rs_id:
                        cut_log.write('Warning: frame id already passed. Skipping this one because lazy.\n')
                        print('Warning: frame id already passed. Skipping this one because lazy.')
                        break

                if video_start_time < 0:
                    continue
                if cut['type'] == 'flight':
                    video_out_file = folder + '/flight' + str(cut['id']) + '.mkv'
                    with open(folder + '/flight' + str(cut['id']) + '.txt', 'w', encoding="utf-8") as flight_start_rs_id_file:
                        flight_start_rs_id_file.write(str(keyframe_rs_id))
                        flight_start_rs_id_file.write('\nThis file is auto generated from cut_moths.py. It contains the rs_id of a few keyframes (video encoding, every 30 frames by default) before the actual start of this flight/insect log.\n')
                else:
                    video_out_file = folder + '/' + cut['type'] + str(cut['id']) + '.mkv'
                video_out_file = video_out_file.replace(os.path.expanduser('~'), '~')
                video_out_file = video_out_file.replace('//', '/')
                cmd = ' -c:v copy -an -ss ' + str(round(video_start_time, 2)) + ' -t ' + str(round(cut['duration'], 2)) + ' ' + video_out_file
                ffmpeg_cmd += cmd
                cut_log.write(cmd + '\n')
                arg_cnt += 1
                if arg_cnt > 500:
                    logger.warning('Too many videos')
                    break

            logger.info(ffmpeg_cmd)
            lb.execute(ffmpeg_cmd, 1, 'process_session')


def filter_cuts(cuts, events):
    cuts_new = []
    for cut in cuts:
        if cut['duration'] > 60:
            cut['duration'] = 60
            cuts_new.append(cut)
        elif cut['duration'] > 0.5:
            cuts_new.append(cut)
        else:
            for event in events:
                if event['video_filename'] == cut['video_filename']:
                    event['video_filename'] == 'NA too short'
    return cuts_new


def cut_video_raw(folder: str, detections: list, flights: list, logger: logging.Logger):
    if not len(detections) and not len(flights):
        return detections, flights
    video_in_fn = folder + '/videoRawLR.mkv'
    if not os.path.exists(video_in_fn):
        logger.warning(video_in_fn + ' did not exists')
        return detections, flights
    frames_fn = folder + '/frames.csv'
    if not os.path.exists(frames_fn):
        logger.warning(frames_fn + ' not found')
        return detections, flights

    hunt_cuts = create_and_associate_hunt_videos(flights, detections)
    monsters = [d for d in detections if d['monster'] == 1 and d['hunt_id'] < 0]
    monster_cuts = create_concated_cuts(monsters, 5, 'anomoly')
    cuts = sorted(monster_cuts + hunt_cuts, key=lambda d: d['rs_id'])
    insects = [d for d in detections if not d['monster'] and d['hunt_id'] < 0]
    associate_insects_to_monsters_cuts(monster_cuts, insects, 5)
    insects_cuts = create_concated_cuts(insects, 1, 'insect')
    cuts = sorted(insects_cuts + monster_cuts + hunt_cuts, key=lambda d: d['rs_id'])
    cuts = associate_detections_to_flight_cuts(cuts, detections)
    cuts = filter_cuts(cuts, detections + flights)

    run_ffmpeg(cuts, frames_fn, folder, video_in_fn)

    if not os.path.exists(lb.keep_videoraw_flag):
        os.remove(video_in_fn)


def process_session(folder: str, dry_run: bool = False):
    logging.basicConfig()
    logger = logging.getLogger('process_session')
    logger.setLevel(logging.DEBUG)
    logger.info("Processing " + folder)
    statuss = []
    detections = []
    flights = []
    flight_sessions = []
    errors = []
    cam_resets = 0

    t_end = str_to_datetime(os.path.basename(os.path.normpath(folder)))

    waited_for_conditions, data_wait_for_conditions, offline_start, _ = process_wait_for_start_condition_status(folder, logger)
    t_start = datetime.max
    if waited_for_conditions:
        statuss.append(data_wait_for_conditions)
        t_start = lb.str_to_datetime(offline_start)

    status_in_folder, mode, operational_log_start = process_system_status_in_folder(folder, logger)

    if status_in_folder != []:
        statuss.append(status_in_folder)
        if lb.str_to_datetime(operational_log_start) < t_start:
            t_start = lb.str_to_datetime(operational_log_start)

    if mode.startswith('Error:'):
        errors.append(mode + '(' + folder + ')')
    elif mode.startswith('Resetting cam'):
        cam_resets = 1
    elif t_start < lb.str_to_datetime('20000101_120000'):
        mode = 'Error: Date from t_start is: ' + lb.datetime_to_str(t_start)  # this seems to be caused by an occasionaly happening realsense problem, that the time just jumps to some extreme number...
        errors.append(mode + '(' + folder + ')')
    else:
        flights_in_folder = []
        if mode == 'x':
            flight_status_in_folder = process_flight_status_in_folder(folder, operational_log_start, mode, logger)
            if len(flight_status_in_folder):
                flight_sessions = flight_status_in_folder
            flights_in_folder = process_flights_in_folder(folder, operational_log_start, logger)
            if flights_in_folder != []:
                flights = flights_in_folder

        detections = process_detections_in_folder(folder, operational_log_start, flights_in_folder, logger)

    if not dry_run:
        try:
            cut_video_raw(folder, detections, flights, logger)
        except Exception as e:  # pylint: disable=broad-except
            logger.error('Error in cutting in ' + folder + '; ' + str(e))

    if mode == 'c' or mode == 'x':
        Path(folder + '/OK').touch()
        data_wait_for_conditions = {"start_datetime": lb.datetime_to_str(t_start),
                                    "end_datetime": lb.datetime_to_str(t_end),
                                    "detections": detections,
                                    "flights": flights,
                                    "flight_sessions": flight_sessions,
                                    "statuss": statuss,
                                    "errors": errors,
                                    "cam_resets": cam_resets
                                    }
        if not dry_run:
            json_fn = folder + '/results.json'
            if exists(json_fn):
                logger.warning('Results json already existed')
            with open(json_fn, 'w', encoding="utf-8") as outfile:
                json.dump(data_wait_for_conditions, outfile)
        logger.info("Processing complete, saved in " + json_fn)
    elif waited_for_conditions:
        Path(folder + '/OK').touch()
        data_wait_for_conditions = {"start_datetime": lb.datetime_to_str(t_start),
                                    "end_datetime": lb.datetime_to_str(t_end),
                                    "statuss": statuss,
                                    "errors": errors,
                                    "cam_resets": cam_resets
                                    }
        if not dry_run:
            json_fn = folder + '/results.json'
            if exists(json_fn):
                logger.warning('Results json already existed')
            with open(json_fn, 'w', encoding="utf-8") as outfile:
                json.dump(data_wait_for_conditions, outfile)
        logger.info("Processing complete, only wait for conditions, saved in " + json_fn)
    else:
        with open(folder + '/junk', 'w', encoding="utf-8") as outfile:
            outfile.write(mode + '\n')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that processes a log folder')
    parser.add_argument('-i', help="Path to the folder", required=True)
    parser.add_argument('--dry-run', help="Do not permanentely change anything", required=False)
    args = parser.parse_args()

    logging.basicConfig()
    logger = logging.getLogger('process_session')
    logger.setLevel(logging.DEBUG)

    if os.path.exists(args.i):
        process_session(args.i, args.dry_run)
    else:
        logger.warning("Folder does not exist: " + args.i)
